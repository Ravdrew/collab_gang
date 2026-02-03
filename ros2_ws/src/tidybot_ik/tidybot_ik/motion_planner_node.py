#!/usr/bin/env python3
"""
TidyBot2 Motion Planner Node

Provides trajectory planning with collision checking and singularity avoidance.
Uses Mink for inverse kinematics with collision limits.

Services:
- /plan_to_target (PlanToTarget): Plan and optionally execute arm motion

Topics subscribed:
- /joint_states (JointState): Current robot configuration

Topics published (for execution):
- /right_arm/cmd (ArmCommand): Right arm commands
- /left_arm/cmd (ArmCommand): Left arm commands
"""

import numpy as np
import mujoco
import mink
from pathlib import Path
from threading import Lock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from tidybot_msgs.msg import ArmCommand
from tidybot_msgs.srv import PlanToTarget


class MotionPlannerNode(Node):
    """Motion planner with IK, collision checking, and singularity detection."""

    # Joint limits (from tidybot_wx250s_bimanual.xml)
    JOINT_LIMITS = {
        'waist': (-3.14159, 3.14159),
        'shoulder': (-1.8849, 1.9897),
        'elbow': (-2.1468, 1.6057),
        'forearm_roll': (-3.14159, 3.14159),
        'wrist_angle': (-1.7453, 2.1468),
        'wrist_rotate': (-3.14159, 3.14159),
    }

    def __init__(self):
        super().__init__('motion_planner')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('ik_dt', 0.002)
        self.declare_parameter('ik_max_iterations', 500)
        self.declare_parameter('position_tolerance', 0.01)  # 1cm
        self.declare_parameter('orientation_tolerance', 0.1)  # ~6 deg
        self.declare_parameter('min_collision_distance', 0.05)  # 5cm

        # Get parameters
        model_path_param = self.get_parameter('model_path').get_parameter_value().string_value
        self.ik_dt = self.get_parameter('ik_dt').get_parameter_value().double_value
        self.ik_max_iterations = self.get_parameter('ik_max_iterations').get_parameter_value().integer_value
        self.position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').get_parameter_value().double_value
        self.min_collision_distance = self.get_parameter('min_collision_distance').get_parameter_value().double_value

        # Find model path
        if model_path_param:
            model_path = Path(model_path_param)
        else:
            # Default: look relative to workspace
            model_path = Path(__file__).parent.parent.parent.parent.parent.parent / \
                'simulation/assets/mujoco/tidybot_wx250s_bimanual.xml'

        if not model_path.exists():
            self.get_logger().error(f'MuJoCo model not found: {model_path}')
            raise FileNotFoundError(f'Model not found: {model_path}')

        # Load MuJoCo model for IK
        self.get_logger().info(f'Loading model: {model_path}')
        self.model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
        self.data = mujoco.MjData(self.model)

        # Initialize Mink configuration
        self.configuration = mink.Configuration(self.model)

        # Joint name mappings (6-DOF WX250s)
        self.arm_joints = {
            'right': ['right_waist', 'right_shoulder', 'right_elbow',
                      'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate'],
            'left': ['left_waist', 'left_shoulder', 'left_elbow',
                     'left_forearm_roll', 'left_wrist_angle', 'left_wrist_rotate'],
        }

        # Get joint qpos addresses
        self.joint_qpos_addrs = {}
        for arm in ['right', 'left']:
            for jname in self.arm_joints[arm]:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                self.joint_qpos_addrs[jname] = self.model.jnt_qposadr[jid]

        # End-effector sites
        self.ee_sites = {
            'right': 'right_pinch_site',
            'left': 'left_pinch_site',
        }

        # Bodies for collision checking
        self.collision_bodies = {
            'right': ['right_upper_arm_link', 'right_forearm_link', 'right_wrist_link', 'right_gripper_link'],
            'left': ['left_upper_arm_link', 'left_forearm_link', 'left_wrist_link', 'left_gripper_link'],
        }

        # Get body IDs
        self.body_ids = {}
        for arm in ['right', 'left']:
            self.body_ids[arm] = []
            for bname in self.collision_bodies[arm]:
                bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, bname)
                self.body_ids[arm].append(bid)

        # Current joint states
        self.current_joint_positions = {}
        self.joint_lock = Lock()

        # Publishers for arm commands (execution)
        self.arm_cmd_pubs = {
            'right': self.create_publisher(ArmCommand, '/right_arm/cmd', 10),
            'left': self.create_publisher(ArmCommand, '/left_arm/cmd', 10),
        }

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Service for planning
        self.plan_service = self.create_service(
            PlanToTarget, '/plan_to_target', self.plan_to_target_callback
        )

        self.get_logger().info('Motion planner initialized')
        self.get_logger().info('Service: /plan_to_target')

    def joint_state_callback(self, msg: JointState):
        """Update current joint positions from joint states."""
        with self.joint_lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.current_joint_positions[name] = msg.position[i]

    def get_arm_joint_positions(self, arm_name: str) -> np.ndarray:
        """Get current joint positions for an arm."""
        with self.joint_lock:
            positions = np.zeros(6)
            for i, jname in enumerate(self.arm_joints[arm_name]):
                positions[i] = self.current_joint_positions.get(jname, 0.0)
            return positions

    def pose_to_se3(self, pose: Pose, use_orientation: bool) -> mink.SE3:
        """Convert geometry_msgs/Pose to mink SE3."""
        position = np.array([pose.position.x, pose.position.y, pose.position.z])

        if use_orientation:
            # Quaternion in wxyz format for mink
            quat = np.array([pose.orientation.w, pose.orientation.x,
                            pose.orientation.y, pose.orientation.z])
            return mink.SE3.from_rotation_and_translation(
                rotation=mink.SO3(quat),
                translation=position
            )
        else:
            # Use identity orientation
            return mink.SE3.from_rotation_and_translation(
                rotation=mink.SO3.identity(),
                translation=position
            )

    def solve_ik(self, arm_name: str, target_pose: mink.SE3,
                 use_orientation: bool, seed: np.ndarray) -> tuple:
        """
        Solve inverse kinematics using Mink.

        Returns: (success, joint_positions, position_error, orientation_error)
        """
        # Create a fresh configuration with current seed
        self.data.qpos[:] = 0
        for i, jname in enumerate(self.arm_joints[arm_name]):
            addr = self.joint_qpos_addrs[jname]
            self.data.qpos[addr] = seed[i]

        mujoco.mj_forward(self.model, self.data)
        self.configuration.update(self.data.qpos)

        # Create frame task for end-effector
        ee_site = self.ee_sites[arm_name]
        position_cost = 1.0
        orientation_cost = 1.0 if use_orientation else 0.0

        ee_task = mink.FrameTask(
            frame_name=ee_site,
            frame_type="site",
            position_cost=position_cost,
            orientation_cost=orientation_cost,
        )
        ee_task.set_target(target_pose)

        # Solve IK iteratively
        for iteration in range(self.ik_max_iterations):
            vel = mink.solve_ik(
                self.configuration,
                [ee_task],
                dt=self.ik_dt,
                solver="quadprog",
                damping=1e-3,
            )
            self.configuration.integrate_inplace(vel, self.ik_dt)

            # Check convergence
            if np.linalg.norm(vel) < 1e-6:
                break

        # Extract solution for the arm
        solution = np.zeros(6)
        for i, jname in enumerate(self.arm_joints[arm_name]):
            addr = self.joint_qpos_addrs[jname]
            solution[i] = self.configuration.q[addr]

        # Clamp to joint limits
        for i, (_, (low, high)) in enumerate(self.JOINT_LIMITS.items()):
            solution[i] = np.clip(solution[i], low, high)

        # Compute errors
        # Update MuJoCo data with solution to get FK
        for i, jname in enumerate(self.arm_joints[arm_name]):
            addr = self.joint_qpos_addrs[jname]
            self.data.qpos[addr] = solution[i]
        mujoco.mj_forward(self.model, self.data)

        # Get actual end-effector pose
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, ee_site)
        actual_pos = self.data.site_xpos[site_id].copy()
        actual_mat = self.data.site_xmat[site_id].reshape(3, 3)

        # Position error
        target_pos = target_pose.translation()
        position_error = np.linalg.norm(actual_pos - target_pos)

        # Orientation error (angle from rotation matrix difference)
        if use_orientation:
            target_mat = target_pose.rotation().as_matrix()
            R_diff = target_mat.T @ actual_mat
            trace = np.trace(R_diff)
            cos_angle = (trace - 1) / 2
            cos_angle = np.clip(cos_angle, -1, 1)
            orientation_error = np.arccos(cos_angle)
        else:
            orientation_error = 0.0

        success = (position_error < self.position_tolerance and
                   (not use_orientation or orientation_error < self.orientation_tolerance))

        return success, solution, position_error, orientation_error

    def compute_jacobian_condition(self, arm_name: str, joint_positions: np.ndarray) -> float:
        """Compute Jacobian condition number at given configuration."""
        # Set joint positions
        for i, jname in enumerate(self.arm_joints[arm_name]):
            addr = self.joint_qpos_addrs[jname]
            self.data.qpos[addr] = joint_positions[i]
        mujoco.mj_forward(self.model, self.data)

        # Get Jacobian using MuJoCo
        site_name = self.ee_sites[arm_name]
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)

        # Allocate Jacobian matrices
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))

        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, site_id)

        # Extract columns for arm joints only
        arm_jac_cols = []
        for jname in self.arm_joints[arm_name]:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            dof_addr = self.model.jnt_dofadr[jid]
            arm_jac_cols.append(dof_addr)

        J_pos = jacp[:, arm_jac_cols]  # 3x6 position Jacobian
        J_rot = jacr[:, arm_jac_cols]  # 3x6 rotation Jacobian

        # Use position Jacobian for condition number (most relevant for manipulation)
        # Add small regularization to avoid numerical issues
        J = J_pos
        try:
            cond = np.linalg.cond(J @ J.T)
        except:
            cond = float('inf')

        return cond

    def check_arm_collision(self, joint_positions_right: np.ndarray,
                           joint_positions_left: np.ndarray) -> tuple:
        """
        Check for collision between the two arms.

        Returns: (collision_free, min_distance)
        """
        # Set both arm configurations
        for i, jname in enumerate(self.arm_joints['right']):
            addr = self.joint_qpos_addrs[jname]
            self.data.qpos[addr] = joint_positions_right[i]
        for i, jname in enumerate(self.arm_joints['left']):
            addr = self.joint_qpos_addrs[jname]
            self.data.qpos[addr] = joint_positions_left[i]

        mujoco.mj_forward(self.model, self.data)

        # Get body positions and check pairwise distances
        min_distance = float('inf')

        for right_bid in self.body_ids['right']:
            right_pos = self.data.xpos[right_bid]
            for left_bid in self.body_ids['left']:
                left_pos = self.data.xpos[left_bid]
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
        target_se3 = self.pose_to_se3(request.target_pose, request.use_orientation)

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

        if condition_number > request.max_condition_number:
            response.success = False
            response.message = f"Near singularity: condition number={condition_number:.1f} > {request.max_condition_number}"
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
            cmd = ArmCommand()
            cmd.joint_positions = solution.tolist()
            cmd.duration = request.duration
            self.arm_cmd_pubs[arm_name].publish(cmd)
            response.executed = True
            self.get_logger().info(f'Executing motion over {request.duration}s')
        else:
            response.executed = False

        return response


def main(args=None):
    rclpy.init(args=args)

    node = MotionPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
