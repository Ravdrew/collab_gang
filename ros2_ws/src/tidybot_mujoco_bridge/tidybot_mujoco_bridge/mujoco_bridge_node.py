#!/usr/bin/env python3
"""
MuJoCo Bridge Node for TidyBot2.

This node wraps MuJoCo simulation and exposes ROS2 topics for:
- Joint states (publish)
- Camera images (publish)
- Odometry (publish)
- TF transforms (publish)
- Joint commands (subscribe)
- Base velocity commands (subscribe)
"""

import os
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState, Image, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Header, Bool
from tf2_ros import TransformBroadcaster

import mujoco
import mujoco.viewer

# Try to import cv_bridge for image publishing
try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class MuJoCoBridgeNode(Node):
    """ROS2 node that bridges MuJoCo simulation to ROS2 topics."""

    # Joint name mapping (order matches MuJoCo actuator order)
    JOINT_NAMES = [
        # Base (3 DOF) - virtual joints, not published
        # 'joint_x', 'joint_y', 'joint_th',
        # Pan-tilt (2 DOF)
        'camera_pan', 'camera_tilt',
        # Right arm (6 DOF)
        'right_waist', 'right_shoulder', 'right_elbow', 'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate',
        # Right gripper (2 DOF)
        'right_left_finger', 'right_right_finger',
        # Left arm (6 DOF)
        'left_waist', 'left_shoulder', 'left_elbow', 'left_forearm_roll', 'left_wrist_angle', 'left_wrist_rotate',
        # Left gripper (2 DOF)
        'left_left_finger', 'left_right_finger',
    ]

    # Actuator names in MuJoCo (same order as JOINT_NAMES but includes base)
    ACTUATOR_NAMES = [
        'joint_x', 'joint_y', 'joint_th',
        'camera_pan', 'camera_tilt',
        'right_waist', 'right_shoulder', 'right_elbow', 'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate',
        'right_left_finger', 'right_right_finger',
        'left_waist', 'left_shoulder', 'left_elbow', 'left_forearm_roll', 'left_wrist_angle', 'left_wrist_rotate',
        'left_left_finger', 'left_right_finger',
    ]

    def __init__(self):
        super().__init__('mujoco_bridge')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('sim_rate', 500.0)  # Hz
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('camera_rate', 30.0)  # Hz
        self.declare_parameter('show_viewer', True)

        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.sim_rate = self.get_parameter('sim_rate').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.camera_rate = self.get_parameter('camera_rate').get_parameter_value().double_value
        show_viewer = self.get_parameter('show_viewer').get_parameter_value().bool_value

        # Find model path if not specified
        if not model_path:
            # Default to the simulation assets
            # When installed, we need to find the repo root from a known location
            # Try environment variable first, then fall back to searching
            repo_root = os.environ.get('TIDYBOT_REPO_ROOT')
            if not repo_root:
                # Search upward for the simulation directory
                search_path = os.path.dirname(os.path.abspath(__file__))
                for _ in range(10):  # Max 10 levels up
                    candidate = os.path.join(search_path, 'simulation', 'assets', 'mujoco')
                    if os.path.isdir(candidate):
                        repo_root = search_path
                        break
                    search_path = os.path.dirname(search_path)

            if repo_root:
                model_path = os.path.join(
                    repo_root, 'simulation', 'assets', 'mujoco', 'scene_wx250s_bimanual.xml'
                )
            else:
                # Last resort: assume we're in ros2_ws inside the repo
                model_path = os.path.expanduser(
                    '~/Documents/collaborative-robotics-2026/simulation/assets/mujoco/scene_wx250s_bimanual.xml'
                )

        self.get_logger().info(f'Loading MuJoCo model from: {model_path}')

        # Load MuJoCo model
        try:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)

            # Load home keyframe if available for stable initial state
            try:
                home_key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, 'home')
                mujoco.mj_resetDataKeyframe(self.model, self.data, home_key_id)
                self.get_logger().info('Loaded home keyframe for stable initial state')
            except Exception:
                self.get_logger().warn('No home keyframe found, using default state')

        except Exception as e:
            self.get_logger().error(f'Failed to load MuJoCo model: {e}')
            raise

        # Build actuator ID lookup
        self.actuator_ids = {}
        for name in self.ACTUATOR_NAMES:
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if actuator_id >= 0:
                self.actuator_ids[name] = actuator_id
            else:
                self.get_logger().warn(f'Actuator not found: {name}')

        # Build joint ID lookup
        self.joint_ids = {}
        # Add base joints (not published but needed for TF)
        for name in ['joint_x', 'joint_y', 'joint_th']:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id >= 0:
                self.joint_ids[name] = joint_id
            else:
                self.get_logger().warn(f'Base joint not found: {name}')
        # Add arm/gripper joints (published in joint_states)
        for name in self.JOINT_NAMES:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id >= 0:
                self.joint_ids[name] = joint_id
                self.get_logger().info(f'Found joint: {name} (id={joint_id})')
            else:
                self.get_logger().error(f'Joint NOT found: {name}')

        # Image bridge
        if HAS_CV_BRIDGE:
            self.cv_bridge = CvBridge()
        else:
            self.get_logger().warn('cv_bridge not available, camera images disabled')

        # Create MuJoCo renderer for camera images
        self.renderer = mujoco.Renderer(self.model, height=480, width=640)

        # State variables
        self.sim_step_count = 0
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_th = np.pi / 2  # Robot model faces -Y, rotate 90° to face +X
        self.cmd_vel = Twist()
        self.current_vel = Twist()  # Smoothed velocity
        self.last_sim_time = None
        self.lock = threading.Lock()

        # Joint position filtering to reduce RViz jitter
        self.filtered_joint_positions = {}
        self.joint_filter_alpha = 1.0  # 1.0 = no filtering, lower = more smoothing

        # Acceleration limits for smooth motion (m/s^2 and rad/s^2)
        self.max_linear_accel = 2.0   # m/s^2 (higher = snappier response)
        self.max_angular_accel = 4.0  # rad/s^2

        # Velocity limits for position control
        self.max_linear_vel = 0.5     # m/s
        self.max_angular_vel = 1.5    # rad/s

        # Position control state
        self.position_control_mode = False
        self.target_pose = None  # Pose2D (x, y, theta)
        self.position_tolerance = 0.02      # meters
        self.orientation_tolerance = 0.05   # radians

        # Position control gains
        self.kp_linear = 2.0   # Proportional gain for linear motion
        self.kp_angular = 3.0  # Proportional gain for angular motion

        # Target positions for all actuators (initialized from data.ctrl which may have keyframe values)
        self.target_ctrl = self.data.ctrl.copy()

        # Ensure gripper positions are valid (minimum 0.015)
        for name in ['right_left_finger', 'right_right_finger', 'left_left_finger', 'left_right_finger']:
            if name in self.actuator_ids:
                idx = self.actuator_ids[name]
                if self.target_ctrl[idx] < 0.015:
                    self.target_ctrl[idx] = 0.015

        # QoS profile - use RELIABLE for RViz compatibility
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', qos)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos)
        self.goal_reached_pub = self.create_publisher(Bool, '/base/goal_reached', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.right_arm_sub = self.create_subscription(
            Float64MultiArray, '/right_arm/joint_cmd', self.right_arm_callback, 10
        )
        self.left_arm_sub = self.create_subscription(
            Float64MultiArray, '/left_arm/joint_cmd', self.left_arm_callback, 10
        )
        self.right_gripper_sub = self.create_subscription(
            Float64MultiArray, '/right_gripper/cmd', self.right_gripper_callback, 10
        )
        self.left_gripper_sub = self.create_subscription(
            Float64MultiArray, '/left_gripper/cmd', self.left_gripper_callback, 10
        )
        self.pan_tilt_sub = self.create_subscription(
            Float64MultiArray, '/camera/pan_tilt_cmd', self.pan_tilt_callback, 10
        )
        # Position control subscriber - send target pose and robot drives there
        self.target_pose_sub = self.create_subscription(
            Pose2D, '/base/target_pose', self.target_pose_callback, 10
        )

        # Timers
        sim_period = 1.0 / self.sim_rate
        self.sim_timer = self.create_timer(sim_period, self.sim_step_callback)

        publish_period = 1.0 / self.publish_rate
        self.publish_timer = self.create_timer(publish_period, self.publish_callback)

        camera_period = 1.0 / self.camera_rate
        self.camera_timer = self.create_timer(camera_period, self.camera_callback)

        # Launch viewer in separate thread if requested
        self.viewer = None
        if show_viewer:
            self.viewer_thread = threading.Thread(target=self.run_viewer, daemon=True)
            self.viewer_thread.start()

        self.get_logger().info('MuJoCo Bridge initialized successfully')
        self.get_logger().info(f'  Sim rate: {self.sim_rate} Hz')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Camera rate: {self.camera_rate} Hz')

    def run_viewer(self):
        """Run MuJoCo viewer in a separate thread."""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.viewer = viewer
            while viewer.is_running():
                with self.lock:
                    viewer.sync()
                # Rate limit viewer to 60 FPS to avoid starving simulation
                time.sleep(1.0 / 60.0)

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands for the mobile base."""
        with self.lock:
            # Velocity commands cancel position control mode
            self.position_control_mode = False
            self.target_pose = None
            self.cmd_vel = msg

    def target_pose_callback(self, msg: Pose2D):
        """Handle position target for the mobile base (go-to-goal)."""
        with self.lock:
            self.position_control_mode = True
            # Target coordinates are in user frame where:
            # - x is forward (+X world, direction camera/arms face)
            # - y is left (+Y world)
            # Robot faces +X at home (MuJoCo theta=π/2), so user frame = world frame for position.
            # For theta: user theta=0 means facing +X = MuJoCo theta π/2
            self.target_pose = Pose2D()
            self.target_pose.x = msg.x
            self.target_pose.y = msg.y
            self.target_pose.theta = msg.theta + np.pi / 2
            self.get_logger().info(
                f'Target: user ({msg.x:.2f}, {msg.y:.2f}, th={msg.theta:.2f}) -> world ({self.target_pose.x:.2f}, {self.target_pose.y:.2f}, th={self.target_pose.theta:.2f})'
            )

    def right_arm_callback(self, msg: Float64MultiArray):
        """Handle joint commands for right arm (6 joints)."""
        if len(msg.data) != 6:
            self.get_logger().warn(f'Right arm command has {len(msg.data)} values, expected 6')
            return
        with self.lock:
            for i, name in enumerate(['right_waist', 'right_shoulder', 'right_elbow',
                                       'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate']):
                if name in self.actuator_ids:
                    self.target_ctrl[self.actuator_ids[name]] = msg.data[i]

    def left_arm_callback(self, msg: Float64MultiArray):
        """Handle joint commands for left arm (6 joints)."""
        if len(msg.data) != 6:
            self.get_logger().warn(f'Left arm command has {len(msg.data)} values, expected 6')
            return
        with self.lock:
            for i, name in enumerate(['left_waist', 'left_shoulder', 'left_elbow',
                                       'left_forearm_roll', 'left_wrist_angle', 'left_wrist_rotate']):
                if name in self.actuator_ids:
                    self.target_ctrl[self.actuator_ids[name]] = msg.data[i]

    def right_gripper_callback(self, msg: Float64MultiArray):
        """Handle gripper command for right gripper (normalized 0-1)."""
        if len(msg.data) < 1:
            return
        # Convert normalized position to finger position
        # 0.0 = open (0.037m), 1.0 = closed (0.015m)
        # Maps [0,1] to [0.037, 0.015] (range of 0.022m)
        pos = 0.037 - msg.data[0] * 0.022
        with self.lock:
            if 'right_left_finger' in self.actuator_ids:
                idx = self.actuator_ids['right_left_finger']
                if abs(self.target_ctrl[idx] - pos) > 0.001:
                    self.get_logger().info(f'Right gripper: {self.target_ctrl[idx]:.4f} -> {pos:.4f}')
                self.target_ctrl[idx] = pos
            if 'right_right_finger' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['right_right_finger']] = pos

    def left_gripper_callback(self, msg: Float64MultiArray):
        """Handle gripper command for left gripper (normalized 0-1)."""
        if len(msg.data) < 1:
            return
        # Convert normalized position to finger position
        # 0.0 = open (0.037m), 1.0 = closed (0.015m)
        pos = 0.037 - msg.data[0] * 0.022
        with self.lock:
            if 'left_left_finger' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['left_left_finger']] = pos
            if 'left_right_finger' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['left_right_finger']] = pos

    def pan_tilt_callback(self, msg: Float64MultiArray):
        """Handle pan-tilt camera commands."""
        if len(msg.data) < 2:
            return
        with self.lock:
            if 'camera_pan' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['camera_pan']] = msg.data[0]
            if 'camera_tilt' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['camera_tilt']] = msg.data[1]

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def compute_position_control_velocity(self):
        """Compute velocity commands to reach target pose."""
        if self.target_pose is None:
            return 0.0, 0.0, 0.0

        # Current pose
        x, y, th = self.base_x, self.base_y, self.base_th

        # Target pose
        tx, ty, tth = self.target_pose.x, self.target_pose.y, self.target_pose.theta

        # Position error in world frame
        dx = tx - x
        dy = ty - y
        distance = np.sqrt(dx * dx + dy * dy)

        # Angle to target (in world frame)
        angle_to_target = np.arctan2(dy, dx)

        # Heading error (difference between current heading and angle to target)
        # MuJoCo theta=0 means facing -Y, so actual heading (0=+X) is theta - π/2
        actual_heading = th - np.pi / 2
        heading_error = self.normalize_angle(angle_to_target - actual_heading)

        # Final orientation error
        # Both tth and th are in MuJoCo theta (π/2 = facing +X)
        orientation_error = self.normalize_angle(tth - th)

        # Check if we've reached the goal
        at_position = distance < self.position_tolerance
        at_orientation = abs(orientation_error) < self.orientation_tolerance

        if at_position and at_orientation:
            # Goal reached - publish and stop
            goal_msg = Bool()
            goal_msg.data = True
            self.goal_reached_pub.publish(goal_msg)
            self.position_control_mode = False
            self.target_pose = None
            self.get_logger().info('Goal reached!')
            return 0.0, 0.0, 0.0

        # Two-phase control: first rotate to face target, then drive + rotate to final
        if not at_position:
            # Phase 1: Drive toward target
            if abs(heading_error) > 0.3:  # ~17 degrees - need to rotate first
                # Rotate in place to face target
                vx = 0.0
                vy = 0.0
                vth = self.kp_angular * heading_error
            else:
                # Drive toward target while correcting heading
                vx = self.kp_linear * distance
                vy = 0.0
                vth = self.kp_angular * heading_error
        else:
            # Phase 2: At position, rotate to final orientation
            vx = 0.0
            vy = 0.0
            vth = self.kp_angular * orientation_error

        # Apply velocity limits
        vx = np.clip(vx, -self.max_linear_vel, self.max_linear_vel)
        vy = np.clip(vy, -self.max_linear_vel, self.max_linear_vel)
        vth = np.clip(vth, -self.max_angular_vel, self.max_angular_vel)

        return vx, vy, vth

    def sim_step_callback(self):
        """Step the MuJoCo simulation."""
        now = time.perf_counter()

        # Use actual elapsed time for accurate integration
        if self.last_sim_time is None:
            dt = 1.0 / self.sim_rate
        else:
            dt = now - self.last_sim_time
            # Clamp dt to avoid huge jumps if callback was delayed
            dt = min(dt, 2.0 / self.sim_rate)
        self.last_sim_time = now

        with self.lock:
            # Get target velocity from position controller or cmd_vel
            if self.position_control_mode:
                target_vx, target_vy, target_vth = self.compute_position_control_velocity()
            else:
                target_vx = self.cmd_vel.linear.x
                target_vy = self.cmd_vel.linear.y
                target_vth = self.cmd_vel.angular.z

            # Apply acceleration limiting for smooth motion
            max_dv_linear = self.max_linear_accel * dt
            max_dv_angular = self.max_angular_accel * dt

            # Ramp linear.x
            dv = target_vx - self.current_vel.linear.x
            if abs(dv) > max_dv_linear:
                dv = max_dv_linear if dv > 0 else -max_dv_linear
            self.current_vel.linear.x += dv

            # Ramp linear.y
            dv = target_vy - self.current_vel.linear.y
            if abs(dv) > max_dv_linear:
                dv = max_dv_linear if dv > 0 else -max_dv_linear
            self.current_vel.linear.y += dv

            # Ramp angular.z
            dv = target_vth - self.current_vel.angular.z
            if abs(dv) > max_dv_angular:
                dv = max_dv_angular if dv > 0 else -max_dv_angular
            self.current_vel.angular.z += dv

            # Use smoothed velocities for integration
            vx = self.current_vel.linear.x
            vy = self.current_vel.linear.y
            vth = self.current_vel.angular.z

            # Transform velocities from robot frame to world frame
            # The robot model faces -Y at theta=0 (arms/camera point in -Y direction),
            # so we offset by -π/2 to convert from ROS convention (forward = +X at theta=0)
            actual_heading = self.base_th - np.pi / 2
            cos_th = np.cos(actual_heading)
            sin_th = np.sin(actual_heading)
            self.base_x += (vx * cos_th - vy * sin_th) * dt
            self.base_y += (vx * sin_th + vy * cos_th) * dt
            self.base_th += vth * dt

            # Set base position actuators
            if 'joint_x' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['joint_x']] = self.base_x
            if 'joint_y' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['joint_y']] = self.base_y
            if 'joint_th' in self.actuator_ids:
                self.target_ctrl[self.actuator_ids['joint_th']] = self.base_th

            # Apply control targets
            self.data.ctrl[:] = self.target_ctrl

            # Step simulation
            mujoco.mj_step(self.model, self.data)

        self.sim_step_count += 1

    def publish_callback(self):
        """Publish joint states, odometry, and TF."""
        now = self.get_clock().now().to_msg()

        with self.lock:
            # One-time debug log
            if not hasattr(self, '_logged_joints'):
                self._logged_joints = True
                self.get_logger().info(f'JOINT_NAMES has {len(self.JOINT_NAMES)} joints: {self.JOINT_NAMES}')
                self.get_logger().info(f'joint_ids has {len(self.joint_ids)} joints: {list(self.joint_ids.keys())}')

            # Publish joint states
            joint_state = JointState()
            joint_state.header.stamp = now
            joint_state.header.frame_id = 'base_link'

            for name in self.JOINT_NAMES:
                if name in self.joint_ids:
                    try:
                        joint_id = self.joint_ids[name]
                        qpos_adr = self.model.jnt_qposadr[joint_id]
                        qvel_adr = self.model.jnt_dofadr[joint_id]

                        # Get raw position and apply low-pass filter
                        raw_pos = float(self.data.qpos[qpos_adr])
                        if name in self.filtered_joint_positions:
                            filtered_pos = (self.joint_filter_alpha * raw_pos +
                                           (1 - self.joint_filter_alpha) * self.filtered_joint_positions[name])
                        else:
                            filtered_pos = raw_pos
                        self.filtered_joint_positions[name] = filtered_pos

                        joint_state.name.append(name)
                        joint_state.position.append(filtered_pos)
                        joint_state.velocity.append(float(self.data.qvel[qvel_adr]))
                        joint_state.effort.append(0.0)  # MuJoCo doesn't directly expose this
                    except Exception as e:
                        self.get_logger().error(f'Error reading joint {name}: {e}')

            self.joint_state_pub.publish(joint_state)

            # Get actual base position from MuJoCo (not commanded position)
            # This ensures TF matches the joint states for consistent visualization
            actual_base_x = 0.0
            actual_base_y = 0.0
            actual_base_th = 0.0
            if 'joint_x' in self.joint_ids:
                qpos_adr = self.model.jnt_qposadr[self.joint_ids['joint_x']]
                actual_base_x = float(self.data.qpos[qpos_adr])
            if 'joint_y' in self.joint_ids:
                qpos_adr = self.model.jnt_qposadr[self.joint_ids['joint_y']]
                actual_base_y = float(self.data.qpos[qpos_adr])
            if 'joint_th' in self.joint_ids:
                qpos_adr = self.model.jnt_qposadr[self.joint_ids['joint_th']]
                actual_base_th = float(self.data.qpos[qpos_adr])

            # Publish odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = actual_base_x
            odom.pose.pose.position.y = actual_base_y
            odom.pose.pose.position.z = 0.0

            # Convert yaw to quaternion
            cy = np.cos(actual_base_th * 0.5)
            sy = np.sin(actual_base_th * 0.5)
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = sy
            odom.pose.pose.orientation.w = cy

            odom.twist.twist = self.current_vel  # Use smoothed velocity

            self.odom_pub.publish(odom)

            # Publish TF: odom -> base_link (using actual MuJoCo position)
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = actual_base_x
            t.transform.translation.y = actual_base_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = sy
            t.transform.rotation.w = cy

            self.tf_broadcaster.sendTransform(t)

    def camera_callback(self):
        """Publish camera images."""
        if not HAS_CV_BRIDGE:
            return

        now = self.get_clock().now().to_msg()

        with self.lock:
            # Render RGB image
            self.renderer.update_scene(self.data, camera='d435_rgb')
            rgb_image = self.renderer.render()
            # Flip vertically and horizontally to match ROS camera convention
            # (MuJoCo camera quaternion causes both vertical and horizontal flip)
            rgb_image = np.fliplr(np.flipud(rgb_image)).copy()

            # Render depth image
            self.renderer.update_scene(self.data, camera='d435_depth')
            self.renderer.enable_depth_rendering()
            depth_image = self.renderer.render()
            self.renderer.disable_depth_rendering()
            # Flip vertically and horizontally to match ROS camera convention
            depth_image = np.fliplr(np.flipud(depth_image)).copy()

        # Publish RGB image
        try:
            rgb_msg = self.cv_bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
            rgb_msg.header.stamp = now
            rgb_msg.header.frame_id = 'camera_color_optical_frame'
            self.rgb_pub.publish(rgb_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish RGB image: {e}')

        # Publish depth image
        try:
            # Convert to 16-bit depth in mm
            depth_mm = (depth_image * 1000).astype(np.uint16)
            depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_mm, encoding='16UC1')
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            self.depth_pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish depth image: {e}')

        # Publish camera info
        camera_info = CameraInfo()
        camera_info.header.stamp = now
        camera_info.header.frame_id = 'camera_color_optical_frame'
        camera_info.width = 640
        camera_info.height = 480
        # Approximate D435 intrinsics (fovy=42 degrees)
        fy = 480 / (2 * np.tan(np.radians(42) / 2))
        fx = fy  # Square pixels
        camera_info.k = [fx, 0.0, 320.0, 0.0, fy, 240.0, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, 320.0, 0.0, 0.0, fy, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(camera_info)

    def destroy_node(self):
        """Clean up resources."""
        if self.viewer is not None:
            self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MuJoCoBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
