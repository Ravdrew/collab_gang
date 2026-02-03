#!/usr/bin/env python3
"""
Image Compression Node for TidyBot2 Network Bridge.

Compresses camera images for efficient network transmission to remote clients.
Uses JPEG compression for RGB and PNG compression for depth images.

Topics:
- Subscribe: /camera/color/image_raw (Image)
- Publish: /camera/color/image_compressed (CompressedImage)
- Subscribe: /camera/depth/image_raw (Image)
- Publish: /camera/depth/image_compressed (CompressedImage)

Configuration:
- jpeg_quality: JPEG compression quality (1-100, default 80)
- png_level: PNG compression level (0-9, default 3)
- target_fps: Target frame rate for compressed output (default 15)

Usage:
    ros2 run tidybot_network_bridge image_compression_node
    ros2 run tidybot_network_bridge image_compression_node --ros-args -p jpeg_quality:=60

Note: For higher performance, consider using the ROS2 image_transport package
      with its built-in compressed transport instead.
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage

# Try to import CV bridge
try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False


class ImageCompressionNode(Node):
    """
    ROS2 node that compresses camera images for network transmission.
    """

    def __init__(self):
        super().__init__('image_compression')

        # Check for OpenCV
        if not HAS_CV:
            self.get_logger().error(
                'OpenCV or cv_bridge not available! Install with:\n'
                '  pip install opencv-python\n'
                '  sudo apt install ros-${ROS_DISTRO}-cv-bridge'
            )
            raise RuntimeError('OpenCV/cv_bridge not available')

        # Declare parameters
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('png_level', 3)
        self.declare_parameter('target_fps', 15.0)
        self.declare_parameter('resize_width', 0)  # 0 = no resize
        self.declare_parameter('resize_height', 0)

        # Get parameters
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.png_level = self.get_parameter('png_level').get_parameter_value().integer_value
        target_fps = self.get_parameter('target_fps').get_parameter_value().double_value
        self.resize_width = self.get_parameter('resize_width').get_parameter_value().integer_value
        self.resize_height = self.get_parameter('resize_height').get_parameter_value().integer_value

        # CV bridge
        self.cv_bridge = CvBridge()

        # Rate limiting
        self.min_period = 1.0 / target_fps
        self.last_color_time = 0.0
        self.last_depth_time = 0.0

        # Statistics
        self.color_count = 0
        self.depth_count = 0
        self.color_bytes_saved = 0
        self.depth_bytes_saved = 0

        # QoS for camera topics
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self.color_compressed_pub = self.create_publisher(
            CompressedImage, '/camera/color/image_compressed', qos
        )
        self.depth_compressed_pub = self.create_publisher(
            CompressedImage, '/camera/depth/image_compressed', qos
        )

        # Subscribers
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, qos
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, qos
        )

        # Stats timer
        self.create_timer(10.0, self.print_stats)

        self.get_logger().info('Image compression node ready')
        self.get_logger().info(f'  JPEG quality: {self.jpeg_quality}')
        self.get_logger().info(f'  PNG level: {self.png_level}')
        self.get_logger().info(f'  Target FPS: {target_fps}')
        if self.resize_width > 0 and self.resize_height > 0:
            self.get_logger().info(f'  Resize to: {self.resize_width}x{self.resize_height}')

    def color_callback(self, msg: Image):
        """Compress and publish color image."""
        # Rate limiting
        now = time.time()
        if now - self.last_color_time < self.min_period:
            return
        self.last_color_time = now

        try:
            # Convert to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Optional resize
            if self.resize_width > 0 and self.resize_height > 0:
                cv_image = cv2.resize(
                    cv_image,
                    (self.resize_width, self.resize_height),
                    interpolation=cv2.INTER_AREA
                )

            # Compress as JPEG
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            _, compressed = cv2.imencode('.jpg', cv_image, encode_params)
            compressed_bytes = compressed.tobytes()

            # Create compressed message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = compressed_bytes

            # Publish
            self.color_compressed_pub.publish(compressed_msg)

            # Statistics
            self.color_count += 1
            original_size = len(msg.data)
            compressed_size = len(compressed_bytes)
            self.color_bytes_saved += original_size - compressed_size

        except Exception as e:
            self.get_logger().warn(f'Failed to compress color image: {e}')

    def depth_callback(self, msg: Image):
        """Compress and publish depth image."""
        # Rate limiting
        now = time.time()
        if now - self.last_depth_time < self.min_period:
            return
        self.last_depth_time = now

        try:
            # Convert to OpenCV image
            # Depth is typically 16UC1 (16-bit unsigned, single channel)
            if msg.encoding == '16UC1':
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            elif msg.encoding == '32FC1':
                # Convert float depth to 16-bit mm
                cv_image_float = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                cv_image = (cv_image_float * 1000).astype(np.uint16)
            else:
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Optional resize
            if self.resize_width > 0 and self.resize_height > 0:
                cv_image = cv2.resize(
                    cv_image,
                    (self.resize_width, self.resize_height),
                    interpolation=cv2.INTER_NEAREST  # Use nearest for depth
                )

            # Compress as PNG (lossless for depth data)
            encode_params = [cv2.IMWRITE_PNG_COMPRESSION, self.png_level]
            _, compressed = cv2.imencode('.png', cv_image, encode_params)
            compressed_bytes = compressed.tobytes()

            # Create compressed message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'png'
            compressed_msg.data = compressed_bytes

            # Publish
            self.depth_compressed_pub.publish(compressed_msg)

            # Statistics
            self.depth_count += 1
            original_size = len(msg.data)
            compressed_size = len(compressed_bytes)
            self.depth_bytes_saved += original_size - compressed_size

        except Exception as e:
            self.get_logger().warn(f'Failed to compress depth image: {e}')

    def print_stats(self):
        """Print compression statistics."""
        if self.color_count > 0 or self.depth_count > 0:
            color_mb = self.color_bytes_saved / (1024 * 1024)
            depth_mb = self.depth_bytes_saved / (1024 * 1024)
            self.get_logger().info(
                f'Compression stats - Color: {self.color_count} frames, {color_mb:.1f} MB saved | '
                f'Depth: {self.depth_count} frames, {depth_mb:.1f} MB saved'
            )


def main(args=None):
    rclpy.init(args=args)

    node = ImageCompressionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
