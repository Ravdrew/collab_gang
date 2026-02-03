#!/usr/bin/env python3
"""
TidyBot2 Motion Planner Node for Real Hardware

Provides trajectory planning with collision checking and singularity avoidance.
Uses Pinocchio for inverse kinematics (no MuJoCo dependency).

Services:
- /plan_to_target (PlanToTarget): Plan and optionally execute arm motion

Topics subscribed:
- /joint_states (JointState): Current robot configuration
- /right_arm/joint_states (JointState): Right arm joint states (xs_sdk)
- /left_arm/joint_states (JointState): Left arm joint states (xs_sdk)

Topics published (for execution):
- /right_arm/commands/joint_group (JointGroupCommand): Right arm commands
- /left_arm/commands/joint_group (JointGroupCommand): Left arm commands

Usage:
    ros2 run tidybot_ik motion_planner_real_node
"""

import numpy as np
from pathlib import Path
from threading import Lock
import subprocess
import tempfile
import os

import pinocchio as pin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from interbotix_xs_msgs.msg import JointGroupCommand
from tidybot_msgs.srv import PlanToTarget


class MotionPlannerRealNode(Node):
    """Motion planner with IK using Pinocchio for real hardware."""

    # Joint limits (from WX250s specs)
    JOINT_LIMITS = {
        'waist': (-3.14159, 3.14159),
        'shoulder': (-1.8849, 1.9897),
        'elbow': (-2.1468, 1.6057),
        'forearm_roll': (-3.14159, 3.14159),
        'wrist_angle': (-1.7453, 2.1468),
        'wrist_rotate': (-3.14159, 3.14159),
    }

    # Default seed position (non-singular) - matches SLEEP_POSE from test_arms_real.py
    # [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
    DEFAULT_SEED = np.array([0.0, -1.0, 0.8, 0.0, 0.5, 0.0])

    def __init__(self):
        super().__init__('motion_planner_real')

        # Declare parameters
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('ik_dt', 0.3)  # Step size for numerical IK
        self.declare_parameter('ik_max_iterations', 200)  # More iterations
        self.declare_parameter('position_tolerance', 0.03)  # 3cm
        self.declare_parameter('orientation_tolerance', 0.1)  # ~6 deg
        self.declare_parameter('min_collision_distance', 0.05)  # 5cm
        self.declare_parameter('ik_damping', 1e-5)  # Less damping for better convergence

        # Get parameters
        urdf_path_param = self.get_parameter('urdf_path').get_parameter_value().string_value
        self.ik_dt = self.get_parameter('ik_dt').get_parameter_value().double_value
        self.ik_max_iterations = self.get_parameter('ik_max_iterations').get_parameter_value().integer_value
        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').get_parameter_value().double_value
        self.min_collision_distance = self.get_parameter('min_collision_distance').get_parameter_value().double_value
        self.ik_damping = self.get_parameter('ik_damping').get_parameter_value().double_value

        # Find URDF path
        if urdf_path_param:
            urdf_path = Path(urdf_path_param)
        else:
            # Default: look in tidybot_description package
            urdf_path = Path(__file__).parent.parent.parent.parent / \
                'tidybot_description/urdf/tidybot_wx250s.urdf.xacro'

        if not urdf_path.exists():
            self.get_logger().error(f'URDF not found: {urdf_path}')
            raise FileNotFoundError(f'URDF not found: {urdf_path}')

        # Process xacro to URDF if needed
        urdf_string = self._process_xacro(urdf_path)

        # Load Pinocchio model from URDF string
        self.get_logger().info(f'Loading URDF: {urdf_path}')
        self.model = pin.buildModelFromXML(urdf_string)
        self.data = self.model.createData()

        self.get_logger().info(f'Pinocchio model loaded: {self.model.nq} DOF, {self.model.nv} velocities')

        # Joint name mappings (6-DOF WX250s)
        self.arm_joints = {
            'right': ['right_waist', 'right_shoulder', 'right_elbow',
                      'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate'],
            'left': ['left_waist', 'left_shoulder', 'left_elbow',
                     'left_forearm_roll', 'left_wrist_angle', 'left_wrist_rotate'],
        }

        # Get joint IDs in Pinocchio model
        self.joint_ids = {}
        for arm in ['right', 'left']:
            for jname in self.arm_joints[arm]:
                if self.model.existJointName(jname):
                    self.joint_ids[jname] = self.model.getJointId(jname)
                else:
                    self.get_logger().warn(f'Joint {jname} not found in model')

        # End-effector frame names
        self.ee_frames = {
            'right': 'right_ee_arm_link',
            'left': 'left_ee_arm_link',
        }

        # Get frame IDs
        self.ee_frame_ids = {}
        for arm, frame_name in self.ee_frames.items():
            if self.model.existFrame(frame_name):
                self.ee_frame_ids[arm] = self.model.getFrameId(frame_name)
            else:
                self.get_logger().warn(f'Frame {frame_name} not found, trying {arm}_pinch_site')
                alt_name = f'{arm}_pinch_site'
                if self.model.existFrame(alt_name):
                    self.ee_frame_ids[arm] = self.model.getFrameId(alt_name)
                    self.ee_frames[arm] = alt_name

        # Collision body names for arm-arm checking
        self.collision_frames = {
            'right': ['right_upper_arm_link', 'right_upper_forearm_link',
                      'right_lower_forearm_link', 'right_wrist_link', 'right_gripper_link'],
            'left': ['left_upper_arm_link', 'left_upper_forearm_link',
                     'left_lower_forearm_link', 'left_wrist_link', 'left_gripper_link'],
        }

        # Get collision frame IDs
        self.collision_frame_ids = {}
        for arm in ['right', 'left']:
            self.collision_frame_ids[arm] = []
            for fname in self.collision_frames[arm]:
                if self.model.existFrame(fname):
                    self.collision_frame_ids[arm].append(self.model.getFrameId(fname))

        # Current joint states
        self.current_joint_positions = {}
        self.joint_lock = Lock()

        # Publishers for arm commands (xs_sdk interface)
        self.arm_cmd_pubs = {
            'right': self.create_publisher(JointGroupCommand, '/right_arm/commands/joint_group', 10),
            'left': self.create_publisher(JointGroupCommand, '/left_arm/commands/joint_group', 10),
        }

        # Subscribers for joint states (both aggregated and per-arm)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(JointState, '/right_arm/joint_states', self.joint_state_callback, 10)
        self.create_subscription(JointState, '/left_arm/joint_states', self.joint_state_callback, 10)

        # Service for planning
        self.plan_service = self.create_service(
            PlanToTarget, '/plan_to_target', self.plan_to_target_callback
        )

        self.get_logger().info('Motion planner (real hardware) initialized')
        self.get_logger().info('Service: /plan_to_target')

    def _process_xacro(self, xacro_path: Path) -> str:
        """Process xacro file to URDF string."""
        if xacro_path.suffix == '.xacro':
            try:
                result = subprocess.run(
                    ['xacro', str(xacro_path)],
                    capture_output=True,
                    text=True,
                    check=True
                )
                return result.stdout
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f'Xacro processing failed: {e.stderr}')
                raise
            except FileNotFoundError:
                self.get_logger().error('xacro command not found. Install with: sudo apt install ros-humble-xacro')
                raise
        else:
            # Read URDF directly
            return xacro_path.read_text()

    def joint_state_callback(self, msg: JointState):
        """Update current joint positions from joint states."""
        with self.joint_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.current_joint_positions[name] = msg.position[i]

    def get_arm_joint_positions(self, arm_name: str, use_default_if_zero: bool = True) -> np.ndarray:
        """Get current joint positions for an arm.

        If use_default_if_zero is True and positions are all near zero,
        returns DEFAULT_SEED to avoid singularity issues.
        """
        with self.joint_lock:
            positions = np.zeros(6)
            for i, jname in enumerate(self.arm_joints[arm_name]):
                positions[i] = self.current_joint_positions.get(jname, 0.0)

            # Use default seed if positions are all near zero (uninitialized or singular config)
            if use_default_if_zero and np.allclose(positions, 0.0, atol=0.01):
                self.get_logger().info(f'Using default seed for {arm_name} arm (current positions near zero)')
                return self.DEFAULT_SEED.copy()

            return positions

    def set_arm_configuration(self, q: np.ndarray, arm_name: str, positions: np.ndarray):
        """Set arm joint positions in configuration vector."""
        for i, jname in enumerate(self.arm_joints[arm_name]):
            if jname in self.joint_ids:
                jid = self.joint_ids[jname]
                # Get the index in q for this joint
                idx = self.model.joints[jid].idx_q
                q[idx] = positions[i]

    def numerical_jacobian(self, q: np.ndarray, arm_name: str, ee_frame_id: int, eps: float = 1e-4) -> np.ndarray:
        """Compute numerical Jacobian for position (3x6 matrix).

        Note: We use numerical Jacobian because Pinocchio's analytical Jacobian
        has issues with floating base models.
        """
        arm_idx_q = []
        for jname in self.arm_joints[arm_name]:
            if jname in self.joint_ids:
                jid = self.joint_ids[jname]
                arm_idx_q.append(self.model.joints[jid].idx_q)

        # Get current position
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pos0 = self.data.oMf[ee_frame_id].translation.copy()

        # Compute numerical derivatives
        J = np.zeros((3, len(arm_idx_q)))
        for i, idx_q in enumerate(arm_idx_q):
            q_plus = q.copy()
            q_plus[idx_q] += eps
            pin.forwardKinematics(self.model, self.data, q_plus)
            pin.updateFramePlacements(self.model, self.data)
            pos_plus = self.data.oMf[ee_frame_id].translation.copy()
            J[:, i] = (pos_plus - pos0) / eps

        return J

    def get_arm_from_configuration(self, q: np.ndarray, arm_name: str) -> np.ndarray:
        """Extract arm joint positions from configuration vector."""
        positions = np.zeros(6)
        for i, jname in enumerate(self.arm_joints[arm_name]):
            if jname in self.joint_ids:
                jid = self.joint_ids[jname]
                idx = self.model.joints[jid].idx_q
                positions[i] = q[idx]
        return positions

    def pose_to_se3(self, pose: Pose) -> pin.SE3:
        """Convert geometry_msgs/Pose to Pinocchio SE3."""
        position = np.array([pose.position.x, pose.position.y, pose.position.z])

        # Quaternion: Pinocchio uses [x, y, z, w] format
        quat = pin.Quaternion(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z
        )

        return pin.SE3(quat.matrix(), position)

    def solve_ik(self, arm_name: str, target_pose: pin.SE3,
                 use_orientation: bool, seed: np.ndarray) -> tuple:
        """
        Solve inverse kinematics using Pinocchio's CLIK (Closed-Loop IK).

        Returns: (success, joint_positions, position_error, orientation_error)
        """
        # Initialize configuration with seed
        q = pin.neutral(self.model)
        self.set_arm_configuration(q, arm_name, seed)

        # Also set the other arm to current position for collision checking
        other_arm = 'left' if arm_name == 'right' else 'right'
        other_positions = self.get_arm_joint_positions(other_arm)
        self.set_arm_configuration(q, other_arm, other_positions)

        # Get end-effector frame ID
        ee_frame_id = self.ee_frame_ids.get(arm_name)
        if ee_frame_id is None:
            return False, seed, float('inf'), float('inf')

        # Get joint q indices for this arm
        arm_idx_q = []
        for jname in self.arm_joints[arm_name]:
            if jname in self.joint_ids:
                jid = self.joint_ids[jname]
                arm_idx_q.append(self.model.joints[jid].idx_q)

        # Target position
        target_position = target_pose.translation

        # Joint limits
        limits = list(self.JOINT_LIMITS.values())

        # IK iteration using numerical Jacobian (position-only for now)
        # Note: Orientation control can be added later with 6D Jacobian
        for iteration in range(self.ik_max_iterations):
            # Forward kinematics
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            # Current position
            current_position = self.data.oMf[ee_frame_id].translation

            # Position error
            pos_error_vec = target_position - current_position
            pos_error = np.linalg.norm(pos_error_vec)

            # Check convergence (position only for now)
            if pos_error < self.position_tolerance:
                break

            # Compute numerical Jacobian (3x6 for position)
            J = self.numerical_jacobian(q, arm_name, ee_frame_id)

            # Damped least squares
            JJT = J @ J.T + self.ik_damping * np.eye(3)

            try:
                v = J.T @ np.linalg.solve(JJT, pos_error_vec)
            except np.linalg.LinAlgError:
                break

            # Update arm joints with joint limit clamping
            for i, idx_q in enumerate(arm_idx_q):
                new_val = q[idx_q] + self.ik_dt * v[i]
                q[idx_q] = np.clip(new_val, limits[i][0], limits[i][1])

        # Compute final orientation error if requested
        ori_error = 0.0
        if use_orientation:
            current_pose = self.data.oMf[ee_frame_id]
            R_diff = target_pose.rotation.T @ current_pose.rotation
            trace = np.trace(R_diff)
            cos_angle = np.clip((trace - 1) / 2, -1, 1)
            ori_error = np.arccos(cos_angle)

        # Extract final solution
        solution = self.get_arm_from_configuration(q, arm_name)

        # Clamp to joint limits
        joint_names = list(self.JOINT_LIMITS.keys())
        for i, jname in enumerate(joint_names):
            low, high = self.JOINT_LIMITS[jname]
            solution[i] = np.clip(solution[i], low, high)

        # Compute final errors using direct position/orientation comparison
        self.set_arm_configuration(q, arm_name, solution)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        current_pose = self.data.oMf[ee_frame_id]

        # Position error (direct Euclidean distance)
        position_error = np.linalg.norm(current_pose.translation - target_pose.translation)

        # Orientation error (rotation angle between frames)
        if use_orientation:
            R_diff = target_pose.rotation.T @ current_pose.rotation
            trace = np.trace(R_diff)
            cos_angle = np.clip((trace - 1) / 2, -1, 1)
            orientation_error = np.arccos(cos_angle)
        else:
            orientation_error = 0.0

        success = (position_error < self.position_tolerance and
                   (not use_orientation or orientation_error < self.orientation_tolerance))

        return success, solution, position_error, orientation_error

    def compute_jacobian_condition(self, arm_name: str, joint_positions: np.ndarray) -> float:
        """Compute Jacobian condition number at given configuration."""
        q = pin.neutral(self.model)
        self.set_arm_configuration(q, arm_name, joint_positions)

        ee_frame_id = self.ee_frame_ids.get(arm_name)
        if ee_frame_id is None:
            return float('inf')

        # Compute numerical Jacobian (3x6 for position)
        J = self.numerical_jacobian(q, arm_name, ee_frame_id)

        try:
            cond = np.linalg.cond(J)
        except:
            cond = float('inf')

        return cond

    def check_arm_collision(self, joint_positions_right: np.ndarray,
                           joint_positions_left: np.ndarray) -> tuple:
        """
        Check for collision between the two arms.

        Returns: (collision_free, min_distance)
        """
        q = pin.neutral(self.model)
        self.set_arm_configuration(q, 'right', joint_positions_right)
        self.set_arm_configuration(q, 'left', joint_positions_left)

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        # Get positions of collision frames and check pairwise distances
        min_distance = float('inf')

        for right_fid in self.collision_frame_ids['right']:
            right_pos = self.data.oMf[right_fid].translation
            for left_fid in self.collision_frame_ids['left']:
                left_pos = self.data.oMf[left_fid].translation
                dist = np.linalg.norm(right_pos - left_pos)
                min_distance = min(min_distance, dist)

        collision_free = min_distance >= self.min_collision_distance
        return collision_free, min_distance

    def plan_to_target_callback(self, request, response):
        """Handle PlanToTarget service request."""
        arm_name = request.arm_name.lower()

        # Validate arm name
        if arm_name not in ['right', 'left']:
            response.success = False
            response.message = f"Invalid arm_name '{request.arm_name}'. Use 'right' or 'left'."
            return response

        self.get_logger().info(f'Planning for {arm_name} arm...')

        # Get current joint positions as seed
        seed = self.get_arm_joint_positions(arm_name)
        other_arm = 'left' if arm_name == 'right' else 'right'
        other_arm_positions = self.get_arm_joint_positions(other_arm)

        # Convert target pose to SE3
        target_se3 = self.pose_to_se3(request.target_pose)

        # Solve IK
        ik_success, solution, pos_error, ori_error = self.solve_ik(
            arm_name, target_se3, request.use_orientation, seed
        )

        response.position_error = pos_error
        response.orientation_error = ori_error
        response.joint_positions = solution.tolist()

        if not ik_success:
            response.success = False
            response.message = f"IK failed: position error={pos_error:.4f}m, orientation error={ori_error:.4f}rad"
            self.get_logger().warn(response.message)
            return response

        # Check singularity (Jacobian condition number)
        condition_number = self.compute_jacobian_condition(arm_name, solution)
        response.condition_number = condition_number

        max_cond = request.max_condition_number if request.max_condition_number > 0 else 100.0
        if condition_number > max_cond:
            response.success = False
            response.message = f"Near singularity: condition number={condition_number:.1f} > {max_cond}"
            self.get_logger().warn(response.message)
            return response

        # Check arm-arm collision
        if arm_name == 'right':
            collision_free, min_dist = self.check_arm_collision(solution, other_arm_positions)
        else:
            collision_free, min_dist = self.check_arm_collision(other_arm_positions, solution)

        if not collision_free:
            response.success = False
            response.message = f"Arm collision detected: min distance={min_dist:.3f}m < {self.min_collision_distance}m"
            self.get_logger().warn(response.message)
            return response

        # Planning succeeded
        response.success = True
        response.message = f"Planning succeeded: pos_err={pos_error:.4f}m, cond={condition_number:.1f}, min_dist={min_dist:.3f}m"
        self.get_logger().info(response.message)

        # Execute if requested
        if request.execute:
            self.execute_trajectory(arm_name, solution, request.duration)
            response.executed = True
            self.get_logger().info(f'Executed motion over {request.duration}s')
        else:
            response.executed = False

        return response

    def execute_trajectory(self, arm_name: str, target: np.ndarray, duration: float):
        """Execute trajectory by interpolating from current to target position."""
        import time

        # Get current joint positions
        start = self.get_arm_joint_positions(arm_name, use_default_if_zero=False)

        # Interpolation parameters
        rate_hz = 50.0  # Command rate
        dt = 1.0 / rate_hz
        num_steps = max(int(duration * rate_hz), 1)

        self.get_logger().info(f'Executing {arm_name} trajectory: {num_steps} steps over {duration}s')

        for i in range(num_steps + 1):
            # Linear interpolation with smooth acceleration (cosine profile)
            t = i / num_steps
            # Smooth start/stop using cosine interpolation
            alpha = 0.5 * (1 - np.cos(np.pi * t))

            # Interpolate joint positions
            q = start + alpha * (target - start)

            # Publish command
            cmd = JointGroupCommand()
            cmd.name = f'{arm_name}_arm'
            cmd.cmd = q.tolist()
            self.arm_cmd_pubs[arm_name].publish(cmd)

            # Wait for next step (except on last iteration)
            if i < num_steps:
                time.sleep(dt)


def main(args=None):
    rclpy.init(args=args)

    node = MotionPlannerRealNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
