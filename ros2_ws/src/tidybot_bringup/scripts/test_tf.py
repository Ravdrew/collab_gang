#!/usr/bin/env python3
"""
TF Validation Test for TidyBot2.

Checks that:
1. All expected TF transforms exist
2. Transforms have valid values (not NaN, normalized quaternions)
3. Transforms actually update when joints move

Usage:
    python3 test_tf.py
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
import math
import time
from interbotix_xs_msgs.msg import JointGroupCommand
from sensor_msgs.msg import JointState


# Initial poses (must match test_arms_real.py)
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]
PAN_TILT_CENTER = [0.0, 0.0]


class TFValidator(Node):
    """Validates TF transforms for TidyBot2."""

    # Expected transforms to check (parent -> child)
    # These must match the actual URDF link names
    EXPECTED_TRANSFORMS = [
        # Base
        ('odom', 'base_link'),

        # Right arm chain (matches URDF: upper_forearm_link and lower_forearm_link)
        ('base_link', 'right_arm_base_link'),
        ('right_arm_base_link', 'right_shoulder_link'),
        ('right_shoulder_link', 'right_upper_arm_link'),
        ('right_upper_arm_link', 'right_upper_forearm_link'),  # elbow joint
        ('right_upper_forearm_link', 'right_lower_forearm_link'),  # forearm_roll joint
        ('right_lower_forearm_link', 'right_wrist_link'),  # wrist_angle joint
        ('right_wrist_link', 'right_gripper_link'),  # wrist_rotate joint
        ('right_gripper_link', 'right_ee_arm_link'),

        # Left arm chain (matches URDF)
        ('base_link', 'left_arm_base_link'),
        ('left_arm_base_link', 'left_shoulder_link'),
        ('left_shoulder_link', 'left_upper_arm_link'),
        ('left_upper_arm_link', 'left_upper_forearm_link'),  # elbow joint
        ('left_upper_forearm_link', 'left_lower_forearm_link'),  # forearm_roll joint
        ('left_lower_forearm_link', 'left_wrist_link'),  # wrist_angle joint
        ('left_wrist_link', 'left_gripper_link'),  # wrist_rotate joint
        ('left_gripper_link', 'left_ee_arm_link'),

        # Camera
        ('base_link', 'camera_link'),
        ('camera_link', 'camera_color_optical_frame'),
        ('camera_link', 'camera_depth_optical_frame'),

        # End-to-end checks
        ('odom', 'right_gripper_link'),
        ('odom', 'left_gripper_link'),
        ('odom', 'camera_color_optical_frame'),
    ]

    def __init__(self):
        super().__init__('tf_validator')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers for xs_sdk joint group commands (real robot interface)
        self.right_arm_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10
        )
        self.left_arm_pub = self.create_publisher(
            JointGroupCommand, '/left_arm/commands/joint_group', 10
        )
        # Pan-tilt is on right_arm namespace
        self.pan_tilt_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10
        )

        # Subscribe to joint states to verify movement
        self.right_joint_states = None
        self.left_joint_states = None
        self.create_subscription(
            JointState, '/right_arm/joint_states', self._right_js_cb, 10
        )
        self.create_subscription(
            JointState, '/left_arm/joint_states', self._left_js_cb, 10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('TidyBot2 TF Validation Test')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting 2 seconds for TF buffer to fill...')

        # Wait for TF buffer to fill, then run validation
        self.create_timer(2.0, self.run_full_validation)

    def _right_js_cb(self, msg):
        self.right_joint_states = msg

    def _left_js_cb(self, msg):
        self.left_joint_states = msg

    def is_valid_transform(self, transform):
        """Check if transform values are valid (not NaN, reasonable magnitude)."""
        t = transform.transform.translation
        r = transform.transform.rotation

        # Check for NaN
        values = [t.x, t.y, t.z, r.x, r.y, r.z, r.w]
        if any(math.isnan(v) for v in values):
            return False, "Contains NaN values"

        # Check for infinite values
        if any(math.isinf(v) for v in values):
            return False, "Contains infinite values"

        # Check quaternion is normalized (within tolerance)
        quat_norm = math.sqrt(r.x**2 + r.y**2 + r.z**2 + r.w**2)
        if abs(quat_norm - 1.0) > 0.01:
            return False, f"Quaternion not normalized (norm={quat_norm:.4f})"

        # Check translation is reasonable (within 10m of origin)
        if abs(t.x) > 10 or abs(t.y) > 10 or abs(t.z) > 10:
            return False, f"Translation too large ({t.x:.2f}, {t.y:.2f}, {t.z:.2f})"

        return True, "OK"

    def get_transform(self, parent, child):
        """Get transform between two frames, return None if not found."""
        try:
            transform = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.5)
            )
            return transform
        except Exception:
            return None

    def get_position(self, transform):
        """Extract position tuple from transform."""
        t = transform.transform.translation
        return (t.x, t.y, t.z)

    def positions_different(self, pos1, pos2, threshold=0.001):
        """Check if two positions are different beyond threshold."""
        diff = math.sqrt(
            (pos1[0] - pos2[0])**2 +
            (pos1[1] - pos2[1])**2 +
            (pos1[2] - pos2[2])**2
        )
        return diff > threshold

    def run_static_validation(self):
        """Run static TF validation checks."""
        self.get_logger().info('')
        self.get_logger().info('PART 1: Static TF Validation')
        self.get_logger().info('-' * 60)

        passed = 0
        failed = 0
        missing = 0

        for parent, child in self.EXPECTED_TRANSFORMS:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.5)
                )

                valid, reason = self.is_valid_transform(transform)

                if valid:
                    t = transform.transform.translation
                    self.get_logger().info(
                        f'[PASS] {parent} -> {child}: '
                        f'({t.x:.3f}, {t.y:.3f}, {t.z:.3f})'
                    )
                    passed += 1
                else:
                    self.get_logger().error(f'[FAIL] {parent} -> {child}: {reason}')
                    failed += 1

            except tf2_ros.LookupException:
                self.get_logger().warn(f'[MISS] {parent} -> {child}: Transform not found')
                missing += 1
            except tf2_ros.ExtrapolationException as e:
                self.get_logger().warn(f'[MISS] {parent} -> {child}: {e}')
                missing += 1
            except Exception as e:
                self.get_logger().error(f'[FAIL] {parent} -> {child}: {e}')
                failed += 1

        self.get_logger().info('')
        self.get_logger().info(f'Static validation: {passed} passed, {failed} failed, {missing} missing')
        return passed, failed, missing

    def move_arm(self, arm_side, positions, wait_time=2.5):
        """Move arm using xs_sdk JointGroupCommand."""
        cmd = JointGroupCommand()
        cmd.name = f'{arm_side}_arm'
        cmd.cmd = positions

        if arm_side == 'right':
            self.right_arm_pub.publish(cmd)
        else:
            self.left_arm_pub.publish(cmd)

        # Wait for movement (don't send more commands while moving)
        time.sleep(wait_time)
        # Spin to update TF
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

    def move_pan_tilt(self, positions, wait_time=2.0):
        """Move pan-tilt using xs_sdk JointGroupCommand."""
        cmd = JointGroupCommand()
        cmd.name = 'pan_tilt'
        cmd.cmd = positions
        self.pan_tilt_pub.publish(cmd)

        time.sleep(wait_time)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

    def get_arm_positions(self, arm_side):
        """Get current joint positions for an arm from joint states."""
        js = self.right_joint_states if arm_side == 'right' else self.left_joint_states
        if js is None:
            return None

        # Expected joint order: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate
        joint_names = [
            f'{arm_side}_waist', f'{arm_side}_shoulder', f'{arm_side}_elbow',
            f'{arm_side}_forearm_roll', f'{arm_side}_wrist_angle', f'{arm_side}_wrist_rotate'
        ]

        positions = []
        for name in joint_names:
            if name in js.name:
                idx = js.name.index(name)
                positions.append(js.position[idx])
            else:
                return None  # Joint not found

        return positions

    def get_pan_tilt_positions(self):
        """Get current pan-tilt positions from right arm joint states."""
        js = self.right_joint_states
        if js is None:
            return None

        positions = []
        for name in ['camera_pan', 'camera_tilt']:
            if name in js.name:
                idx = js.name.index(name)
                positions.append(js.position[idx])
            else:
                return None

        return positions

    def run_dynamic_validation(self):
        """Test that TF updates when joints move."""
        self.get_logger().info('')
        self.get_logger().info('PART 2: Dynamic TF Validation (movement test)')
        self.get_logger().info('-' * 60)

        dynamic_passed = 0
        dynamic_failed = 0
        dynamic_skipped = 0

        # Check if we have joint states (indicates arms are connected)
        # Spin a few times to make sure we get joint states
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        has_right = self.right_joint_states is not None
        has_left = self.left_joint_states is not None

        if not has_right and not has_left:
            self.get_logger().warn('No joint states received - skipping dynamic tests')
            self.get_logger().warn('Make sure real.launch.py is running with arms enabled')
            return 0, 0, 3

        # Move to initial position (SLEEP_POSE) before testing
        self.get_logger().info('Moving to initial position (SLEEP_POSE)...')
        if has_right:
            self.move_arm('right', SLEEP_POSE)
        if has_left:
            self.move_arm('left', SLEEP_POSE)
        self.move_pan_tilt(PAN_TILT_CENTER)

        # Store initial positions (should now be SLEEP_POSE)
        right_initial = SLEEP_POSE if has_right else None
        left_initial = SLEEP_POSE if has_left else None
        pan_tilt_initial = PAN_TILT_CENTER

        if right_initial:
            self.get_logger().info(f'Right arm initial: {[f"{p:.2f}" for p in right_initial]}')
        if left_initial:
            self.get_logger().info(f'Left arm initial: {[f"{p:.2f}" for p in left_initial]}')
        self.get_logger().info(f'Pan-tilt initial: {[f"{p:.2f}" for p in pan_tilt_initial]}')

        # Test 1: Right arm waist movement
        self.get_logger().info('')
        self.get_logger().info('Test: Right arm waist rotation')

        if not has_right:
            self.get_logger().warn('[SKIP] Right arm not available')
            dynamic_skipped += 1
        else:
            tf_before = self.get_transform('odom', 'right_gripper_link')
            if tf_before is None:
                self.get_logger().warn('[SKIP] Cannot get right_gripper_link transform')
                dynamic_skipped += 1
            else:
                pos_before = self.get_position(tf_before)
                self.get_logger().info(f'  Before: ({pos_before[0]:.3f}, {pos_before[1]:.3f}, {pos_before[2]:.3f})')

                # Create test position: add 0.3 to waist, keep other joints at initial
                test_pos = list(right_initial) if right_initial else [0.0] * 6
                test_pos[0] += 0.3  # Add to waist

                self.get_logger().info('  Sending waist rotation command (+0.3 rad)...')
                self.move_arm('right', test_pos)

                tf_after = self.get_transform('odom', 'right_gripper_link')
                if tf_after is None:
                    self.get_logger().error('[FAIL] Lost transform after movement')
                    dynamic_failed += 1
                else:
                    pos_after = self.get_position(tf_after)
                    self.get_logger().info(f'  After:  ({pos_after[0]:.3f}, {pos_after[1]:.3f}, {pos_after[2]:.3f})')

                    if self.positions_different(pos_before, pos_after):
                        self.get_logger().info('[PASS] TF updated after arm movement')
                        dynamic_passed += 1
                    else:
                        self.get_logger().error('[FAIL] TF did not change after arm movement')
                        dynamic_failed += 1

                # Return arm to initial position
                if right_initial:
                    self.get_logger().info('  Returning to initial position...')
                    self.move_arm('right', right_initial)

        # Test 2: Left arm shoulder movement
        self.get_logger().info('')
        self.get_logger().info('Test: Left arm shoulder movement')

        if not has_left:
            self.get_logger().warn('[SKIP] Left arm not available')
            dynamic_skipped += 1
        else:
            tf_before = self.get_transform('odom', 'left_gripper_link')
            if tf_before is None:
                self.get_logger().warn('[SKIP] Cannot get left_gripper_link transform')
                dynamic_skipped += 1
            else:
                pos_before = self.get_position(tf_before)
                self.get_logger().info(f'  Before: ({pos_before[0]:.3f}, {pos_before[1]:.3f}, {pos_before[2]:.3f})')

                # Create test position: add 0.3 to shoulder, keep other joints at initial
                test_pos = list(left_initial) if left_initial else [0.0] * 6
                test_pos[1] += 0.3  # Add to shoulder

                self.get_logger().info('  Sending shoulder movement command (+0.3 rad)...')
                self.move_arm('left', test_pos)

                tf_after = self.get_transform('odom', 'left_gripper_link')
                if tf_after is None:
                    self.get_logger().error('[FAIL] Lost transform after movement')
                    dynamic_failed += 1
                else:
                    pos_after = self.get_position(tf_after)
                    self.get_logger().info(f'  After:  ({pos_after[0]:.3f}, {pos_after[1]:.3f}, {pos_after[2]:.3f})')

                    if self.positions_different(pos_before, pos_after):
                        self.get_logger().info('[PASS] TF updated after arm movement')
                        dynamic_passed += 1
                    else:
                        self.get_logger().error('[FAIL] TF did not change after arm movement')
                        dynamic_failed += 1

                # Return to initial position
                if left_initial:
                    self.get_logger().info('  Returning to initial position...')
                    self.move_arm('left', left_initial)

        # Test 3: Camera pan-tilt movement
        self.get_logger().info('')
        self.get_logger().info('Test: Camera pan movement')

        tf_before = self.get_transform('base_link', 'camera_link')
        if tf_before is None:
            self.get_logger().warn('[SKIP] Cannot get camera_link transform')
            dynamic_skipped += 1
        else:
            pos_before = self.get_position(tf_before)
            self.get_logger().info(f'  Before: ({pos_before[0]:.3f}, {pos_before[1]:.3f}, {pos_before[2]:.3f})')

            # Create test position: add 0.3 to pan, keep tilt at initial
            test_pos = list(pan_tilt_initial) if pan_tilt_initial else [0.0, 0.0]
            test_pos[0] += 0.3  # Add to pan

            self.get_logger().info('  Sending pan command (+0.3 rad)...')
            self.move_pan_tilt(test_pos)

            tf_after = self.get_transform('base_link', 'camera_link')
            if tf_after is None:
                self.get_logger().error('[FAIL] Lost transform after movement')
                dynamic_failed += 1
            else:
                pos_after = self.get_position(tf_after)
                self.get_logger().info(f'  After:  ({pos_after[0]:.3f}, {pos_after[1]:.3f}, {pos_after[2]:.3f})')

                if self.positions_different(pos_before, pos_after):
                    self.get_logger().info('[PASS] TF updated after pan-tilt movement')
                    dynamic_passed += 1
                else:
                    self.get_logger().error('[FAIL] TF did not change after pan-tilt movement')
                    dynamic_failed += 1

            # Return to initial position
            if pan_tilt_initial:
                self.get_logger().info('  Returning to initial position...')
                self.move_pan_tilt(pan_tilt_initial)

        self.get_logger().info('')
        self.get_logger().info(f'Dynamic validation: {dynamic_passed} passed, {dynamic_failed} failed, {dynamic_skipped} skipped')
        return dynamic_passed, dynamic_failed, dynamic_skipped

    def run_full_validation(self):
        """Run complete validation suite."""
        # Run static checks
        static_passed, static_failed, static_missing = self.run_static_validation()

        # Run dynamic checks
        dynamic_passed, dynamic_failed, dynamic_skipped = self.run_dynamic_validation()

        # Final summary
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('FINAL SUMMARY')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Static:  {static_passed} passed, {static_failed} failed, {static_missing} missing')
        self.get_logger().info(f'Dynamic: {dynamic_passed} passed, {dynamic_failed} failed, {dynamic_skipped} skipped')
        self.get_logger().info('')

        total_failed = static_failed + dynamic_failed
        if total_failed == 0 and static_missing == 0:
            self.get_logger().info('All TF validation tests PASSED!')
        elif total_failed > 0:
            self.get_logger().error(f'{total_failed} tests FAILED!')
        else:
            self.get_logger().warn('Some transforms missing (may be OK depending on config)')

        self.get_logger().info('=' * 60)
        self.get_logger().info('TF validation complete.')

        # Wait a moment before exiting to let any pending commands finish
        time.sleep(1.0)
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = TFValidator()

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
