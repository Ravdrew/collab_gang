#!/usr/bin/env python3

# Naive baseline state estimator for TidyBot navigation.
# This node is intentionally simple so sensor topics and parameters can be plugged in later.
# Current flow:
# 1) Read base odometry as the main motion input.
# 2) Optionally read IMU yaw-rate for heading prediction.
# 3) Optionally blend absolute vision pose updates when available.
# 4) Publish estimated odometry for downstream navigation nodes.

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose2D


# Keep yaw angles in the [-pi, pi] range.
def wrap_angle(angle_radians):
    return (angle_radians + math.pi) % (2.0 * math.pi) - math.pi


# Convert planar quaternion values into yaw.
def yaw_from_quaternion(orientation):
    return 2.0 * math.atan2(orientation.z, orientation.w)


# Convert yaw into quaternion z/w values.
def quaternion_from_yaw(yaw_radians):
    qz = math.sin(0.5 * yaw_radians)
    qw = math.cos(0.5 * yaw_radians)
    return qz, qw


class BaseStateEstimator(Node):

    # Configure parameters, internal state, topics, and timer.
    def __init__(self):
        super().__init__('base_state_estimator')

        # NOTE: Sensor-specific topics are not finalized yet.
        # These defaults are placeholders and should be updated per robot setup.
        self.declare_parameter('odom_input_topic', '/odom')
        self.declare_parameter('imu_input_topic', '/imu/data')
        self.declare_parameter('vision_pose_input_topic', '/vision/base_pose_2d')
        self.declare_parameter('odom_output_topic', '/odom_est')
        self.declare_parameter('pose2d_output_topic', '/odom_est_pose')
        self.declare_parameter('odom_heading_offset', -math.pi / 2.0)

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)

        self.declare_parameter('use_imu', True)
        self.declare_parameter('use_vision', True)
        self.declare_parameter('vision_position_blend', 0.30)
        self.declare_parameter('vision_yaw_blend', 0.40)
        self.declare_parameter('max_vision_jump_m', 1.5)

        self.odom_input_topic = self.get_parameter('odom_input_topic').value
        self.imu_input_topic = self.get_parameter('imu_input_topic').value
        self.vision_pose_input_topic = self.get_parameter('vision_pose_input_topic').value
        self.odom_output_topic = self.get_parameter('odom_output_topic').value
        self.pose2d_output_topic = self.get_parameter('pose2d_output_topic').value
        self.odom_heading_offset = float(self.get_parameter('odom_heading_offset').value)

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.use_imu = bool(self.get_parameter('use_imu').value)
        self.use_vision = bool(self.get_parameter('use_vision').value)
        self.vision_position_blend = float(self.get_parameter('vision_position_blend').value)
        self.vision_yaw_blend = float(self.get_parameter('vision_yaw_blend').value)
        self.max_vision_jump_m = float(self.get_parameter('max_vision_jump_m').value)

        self.state_x = 0.0
        self.state_y = 0.0
        self.state_yaw = 0.0
        self.state_initialized = False

        self.local_velocity_x = 0.0
        self.local_velocity_y = 0.0
        self.odom_yaw_rate = 0.0
        self.imu_yaw_rate = 0.0
        self.has_imu = False

        self.last_prediction_time = None

        self.estimated_odom_publisher = self.create_publisher(Odometry, self.odom_output_topic, 10)
        self.estimated_pose2d_publisher = self.create_publisher(Pose2D, self.pose2d_output_topic, 10)

        self.odom_subscriber = self.create_subscription(
            Odometry, self.odom_input_topic, self.odom_callback, 10
        )
        self.imu_subscriber = self.create_subscription(
            Imu, self.imu_input_topic, self.imu_callback, 10
        )
        self.vision_pose_subscriber = self.create_subscription(
            Pose2D, self.vision_pose_input_topic, self.vision_pose_callback, 10
        )

        self.update_timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

        self.get_logger().info(
            f'Estimator ready | odom_in={self.odom_input_topic}, '
            f'imu_in={self.imu_input_topic}, vision_in={self.vision_pose_input_topic}, '
            f'odom_out={self.odom_output_topic}'
        )

    # Read odometry and initialize state if this is the first message.
    # Uses the same quaternion-to-yaw pattern as our_trajectory_tracking.py.
    def odom_callback(self, msg):
        self.local_velocity_x = float(msg.twist.twist.linear.x)
        self.local_velocity_y = float(msg.twist.twist.linear.y)
        self.odom_yaw_rate = float(msg.twist.twist.angular.z)

        if not self.state_initialized:
            self.state_x = float(msg.pose.pose.position.x)
            self.state_y = float(msg.pose.pose.position.y)
            odom_theta = yaw_from_quaternion(msg.pose.pose.orientation)
            self.state_yaw = wrap_angle(odom_theta + self.odom_heading_offset)
            self.state_initialized = True
            self.last_prediction_time = self.get_clock().now()

    # Read IMU yaw-rate. This is optional and only used if use_imu is true.
    def imu_callback(self, msg):
        self.imu_yaw_rate = float(msg.angular_velocity.z)
        self.has_imu = True

    # Read absolute vision pose and blend it into the estimated state.
    def vision_pose_callback(self, msg):
        if not self.use_vision:
            return

        if not self.state_initialized:
            self.state_x = float(msg.x)
            self.state_y = float(msg.y)
            self.state_yaw = wrap_angle(float(msg.theta))
            self.state_initialized = True
            self.last_prediction_time = self.get_clock().now()
            return

        dx = float(msg.x) - self.state_x
        dy = float(msg.y) - self.state_y
        position_jump = math.hypot(dx, dy)

        if position_jump > self.max_vision_jump_m:
            return

        yaw_error = wrap_angle(float(msg.theta) - self.state_yaw)

        self.state_x += self.vision_position_blend * dx
        self.state_y += self.vision_position_blend * dy
        self.state_yaw = wrap_angle(self.state_yaw + self.vision_yaw_blend * yaw_error)

    # Run one prediction/update cycle and publish estimated state.
    def timer_callback(self):
        if not self.state_initialized:
            return

        now_time = self.get_clock().now()
        self.predict_state(now_time)
        self.publish_estimated_state(now_time)

    # Predict current pose from velocity measurements.
    def predict_state(self, now_time):
        if self.last_prediction_time is None:
            self.last_prediction_time = now_time
            return

        dt = (now_time - self.last_prediction_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        if self.use_imu and self.has_imu:
            yaw_rate = self.imu_yaw_rate
        else:
            yaw_rate = self.odom_yaw_rate

        world_vx = (
            self.local_velocity_x * math.cos(self.state_yaw)
            - self.local_velocity_y * math.sin(self.state_yaw)
        )
        world_vy = (
            self.local_velocity_x * math.sin(self.state_yaw)
            + self.local_velocity_y * math.cos(self.state_yaw)
        )

        self.state_x += world_vx * dt
        self.state_y += world_vy * dt
        self.state_yaw = wrap_angle(self.state_yaw + yaw_rate * dt)

        self.last_prediction_time = now_time

    # Publish Odometry and Pose2D outputs for navigation consumers.
    def publish_estimated_state(self, now_time):
        qz, qw = quaternion_from_yaw(self.state_yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        odom_msg.pose.pose.position.x = self.state_x
        odom_msg.pose.pose.position.y = self.state_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = self.local_velocity_x
        odom_msg.twist.twist.linear.y = self.local_velocity_y
        if self.use_imu and self.has_imu:
            odom_msg.twist.twist.angular.z = self.imu_yaw_rate
        else:
            odom_msg.twist.twist.angular.z = self.odom_yaw_rate

        # Naive fixed covariance values. Tune later with real logs.
        odom_msg.pose.covariance[0] = 0.05 * 0.05
        odom_msg.pose.covariance[7] = 0.05 * 0.05
        odom_msg.pose.covariance[35] = 0.10 * 0.10

        odom_msg.twist.covariance[0] = 0.10 * 0.10
        odom_msg.twist.covariance[7] = 0.10 * 0.10
        odom_msg.twist.covariance[35] = 0.20 * 0.20

        self.estimated_odom_publisher.publish(odom_msg)

        pose2d_msg = Pose2D()
        pose2d_msg.x = self.state_x
        pose2d_msg.y = self.state_y
        pose2d_msg.theta = self.state_yaw
        self.estimated_pose2d_publisher.publish(pose2d_msg)


# Start the state estimator node.
def main(args=None):
    rclpy.init(args=args)
    node = BaseStateEstimator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
