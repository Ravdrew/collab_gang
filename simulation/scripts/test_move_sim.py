#!/usr/bin/env python3
"""
TidyBot2 with Bimanual WX250s Arms - MuJoCo Simulation Demo

This script demonstrates how to:
1. Load the TidyBot2 mobile robot with dual WX250s 6-DOF arms in MuJoCo
2. Control the mobile base (x, y, rotation) with configurable speed
3. Control both arm joints (left/right: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate)
4. Control both grippers (open/close)
5. Control the 2-DOF pan-tilt camera system
6. Capture RGB and depth images from the simulated Intel RealSense D435

The demo drives the robot forward 1 meter while performing a synchronized arm wave motion,
then demonstrates camera pan-tilt control and captures images at multiple positions.

Speed Control:
    Base movement speed is controlled by incrementally moving the target position
    rather than jumping directly to the goal. This gives smooth, predictable motion
    independent of the actuator gains defined in the XML.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
from pathlib import Path

# For saving captured images
try:
    from PIL import Image
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False
    print("Note: PIL not installed. Images will be saved as numpy arrays (.npy)")
    print("      Install with: pip install pillow")


# ==============================================================================
# CONFIGURATION - Adjust these values to change robot behavior
# ==============================================================================

# --- Base Movement Speed ---
# These control how fast the base moves by limiting the rate of target position change.
# The actual motion will be smooth due to the position controller in the actuator.
BASE_LINEAR_SPEED = 0.3      # meters per second (forward/backward and strafe)
BASE_ANGULAR_SPEED = 1.0     # radians per second (rotation)

# --- Motion Targets ---
# Final goal positions for the demo
TARGET_X_POSITION = 0.75      # meters (how far forward to drive)
TARGET_Y_POSITION = 0.0      # meters (lateral position, 0 = no strafe)
TARGET_THETA = 0.0           # radians (final rotation, 0 = no rotation)

# --- Arm Motion Timing ---
ARM_RAISE_DURATION = 3.0     # seconds to raise arms
ARM_WAVE_DURATION = 3.0      # seconds to wave
ARM_WAVE_FREQUENCY = 2.0     # Hz (oscillations per second)
ARM_WAVE_AMPLITUDE = 0.5     # radians (wave amplitude)

# --- Camera Pan-Tilt Settings ---
# The pan-tilt system has 2 degrees of freedom:
#   - Pan:  rotates camera left/right, range: -1.57 to +1.57 rad (-90° to +90°)
#   - Tilt: rotates camera down/up,    range: -1.57 to +1.31 rad (-90° to +75°)
CAMERA_PAN_SPEED = 0.5       # radians per second
CAMERA_TILT_SPEED = 0.5      # radians per second

# Positions to capture images at: list of (pan, tilt) in radians
# The camera will move to each position and capture RGB + depth images
CAMERA_CAPTURE_POSITIONS = [
    (0.0, 0.0),       # Center (looking straight ahead)
    (-0.5, 0.0),      # Pan left 30°
    (0.5, 0.0),       # Pan right 30°
    (0.0, -0.4),      # Tilt down 23°
    (0.0, 0.3),       # Tilt up 17°
]

# --- Image Capture Settings ---
CAPTURE_WIDTH = 640          # Image width in pixels
CAPTURE_HEIGHT = 480         # Image height in pixels


# ==============================================================================
# IMAGE CAPTURE FUNCTIONS
# ==============================================================================

def capture_rgb_image(renderer, data, camera_name="d435_rgb"):
    """
    Capture an RGB image from the specified camera.

    Args:
        renderer: MuJoCo Renderer object
        data: MuJoCo simulation data
        camera_name: Name of camera defined in XML (default: "d435_rgb")

    Returns:
        numpy array of shape (height, width, 3) with RGB values 0-255
    """
    # Point renderer at the named camera and render the scene
    renderer.update_scene(data, camera=camera_name)

    # Render the scene - returns RGB image as numpy array (height, width, 3)
    rgb = renderer.render()

    # MuJoCo renders images upside-down and left-right mirrored due to camera quaternion,
    # so flip both vertically and horizontally
    rgb = np.fliplr(np.flipud(rgb))

    return rgb


def capture_depth_image(renderer, data, camera_name="d435_depth"):
    """
    Capture a depth image from the specified camera.

    Args:
        renderer: MuJoCo Renderer object
        data: MuJoCo simulation data
        camera_name: Name of camera defined in XML (default: "d435_depth")

    Returns:
        numpy array of shape (height, width) with depth values (0-1 normalized)
    """
    # Point renderer at the named camera
    renderer.update_scene(data, camera=camera_name)

    # Enable depth rendering mode
    renderer.enable_depth_rendering()

    # Render and get depth buffer - returns (height, width) array
    depth = renderer.render()

    # Disable depth rendering for next RGB capture
    renderer.disable_depth_rendering()

    # Flip vertically and horizontally to match RGB orientation
    depth = np.fliplr(np.flipud(depth))

    return depth


def save_image(image, filepath):
    """
    Save an image to disk.

    Args:
        image: numpy array (RGB or depth)
        filepath: Path to save the image
    """
    filepath = Path(filepath)

    if PIL_AVAILABLE:
        if len(image.shape) == 3:
            # RGB image - save directly
            img = Image.fromarray(image)
            img.save(filepath)
        else:
            # Depth image - normalize to 0-255 for visualization
            depth_normalized = image - image.min()
            if depth_normalized.max() > 0:
                depth_normalized = (depth_normalized / depth_normalized.max() * 255)
            img = Image.fromarray(depth_normalized.astype(np.uint8), mode='L')
            img.save(filepath)
        print(f"    Saved: {filepath}")
    else:
        # Save as numpy array if PIL not available
        npy_path = filepath.with_suffix('.npy')
        np.save(npy_path, image)
        print(f"    Saved: {npy_path}")


def main():
    # ==========================================================================
    # STEP 1: Load the MuJoCo model
    # ==========================================================================
    # The scene XML file includes the robot, floor, and lighting
    # Get path relative to this script's location
    script_dir = Path(__file__).parent
    model_path = script_dir / "../assets/mujoco/scene_wx250s_bimanual.xml"
    model_path = model_path.resolve()  # Convert to absolute path
    model = mujoco.MjModel.from_xml_path(str(model_path))  # Static model definition
    data = mujoco.MjData(model)  # Dynamic simulation state (positions, velocities, etc.)

    # Get simulation timestep (used for speed calculations)
    dt = model.opt.timestep

    # ==========================================================================
    # STEP 2: Get joint IDs for reading robot state
    # ==========================================================================
    # Joints let us READ the current position/velocity of each degree of freedom.
    # Use data.qpos[model.jnt_qposadr[joint_id]] to get the current joint position.

    # --- Mobile Base Joints ---
    # The base has 3 degrees of freedom: x position, y position, and rotation (theta)
    base_joint_x = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_x")      # Forward/backward (meters)
    base_joint_y = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_y")      # Left/right strafe (meters)
    base_joint_theta = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_th") # Rotation (radians)

    # ==========================================================================
    # STEP 3: Get actuator IDs for controlling the robot
    # ==========================================================================
    # Actuators let us COMMAND the robot to move. Set data.ctrl[actuator_id] to
    # send a control signal. For position-controlled joints, this is the target position.

    # --- Mobile Base Actuators ---
    # These control the base position directly (position control mode)
    base_ctrl_x = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "joint_x")      # Target x position (meters)
    base_ctrl_y = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "joint_y")      # Target y position (meters)
    base_ctrl_theta = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "joint_th") # Target rotation (radians)

    # --- Right WX250s Arm Actuators (6 joints) ---
    # The arm has 6 degrees of freedom, from base to end-effector:
    right_arm_ctrl_waist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_waist")            # Rotates entire arm left/right
    right_arm_ctrl_shoulder = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_shoulder")      # Raises/lowers upper arm
    right_arm_ctrl_elbow = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_elbow")            # Bends the elbow
    right_arm_ctrl_forearm_roll = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_forearm_roll")  # Rotates the forearm
    right_arm_ctrl_wrist_angle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wrist_angle")    # Bends wrist up/down
    right_arm_ctrl_wrist_rotate = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_wrist_rotate")  # Rotates wrist

    # --- Right Gripper Actuators (2 finger joints) ---
    right_gripper_ctrl_left_finger = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_left_finger")  # Left finger (0.015 = open, 0.037 = closed)
    right_gripper_ctrl_right_finger = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_right_finger")  # Right finger (0.015 = open, 0.037 = closed)

    # --- Left WX250s Arm Actuators (6 joints) ---
    left_arm_ctrl_waist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_waist")
    left_arm_ctrl_shoulder = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_shoulder")
    left_arm_ctrl_elbow = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_elbow")
    left_arm_ctrl_forearm_roll = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_forearm_roll")
    left_arm_ctrl_wrist_angle = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wrist_angle")
    left_arm_ctrl_wrist_rotate = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_wrist_rotate")

    # --- Left Gripper Actuators (2 finger joints) ---
    left_gripper_ctrl_left_finger = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_left_finger")  # Left finger (0.015 = open, 0.037 = closed)
    left_gripper_ctrl_right_finger = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_right_finger")  # Right finger (0.015 = open, 0.037 = closed)

    # --- Pan-Tilt Camera Actuators (2 DOF) ---
    # The camera is mounted on a 2-axis gimbal allowing it to look around
    camera_pan_ctrl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "camera_pan")    # Left/right rotation (radians)
    camera_tilt_ctrl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "camera_tilt")  # Up/down rotation (radians)

    # ==========================================================================
    # STEP 4: Create renderer for image capture
    # ==========================================================================
    # The renderer allows us to capture images from any camera defined in the XML
    renderer = mujoco.Renderer(model, CAPTURE_HEIGHT, CAPTURE_WIDTH)

    # Create output directory for captured images
    output_dir = script_dir / "captures"
    output_dir.mkdir(exist_ok=True)

    # ==========================================================================
    # STEP 5: Initialize robot to home position
    # ==========================================================================
    mujoco.mj_resetData(model, data)

    # Set initial joint positions (data.qpos stores all joint positions)
    # Order matches the order joints are defined in the XML
    # qpos order: base(3) + pan_tilt(2) + right_arm(6) + right_gripper(2) + left_arm(6) + left_gripper(2) = 21
    home_qpos = [
        # Mobile base (3 DOF)
        0, 0, 0,              # x=0m, y=0m, theta=0rad (at origin, facing forward)
        # Pan-tilt camera (2 DOF)
        0, 0,                 # pan=0rad, tilt=0rad (camera facing forward)
        # Right WX250s arm (6 DOF)
        0, 0, 0, 0, 0, 0,     # waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate
        # Right gripper fingers (2 DOF)
        0.015, 0.015,         # left_finger, right_finger (0.015 = open)
        # Left WX250s arm (6 DOF)
        0, 0, 0, 0, 0, 0,     # waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate
        # Left gripper fingers (2 DOF)
        0.015, 0.015          # left_finger, right_finger (0.015 = open)
    ]
    data.qpos[:len(home_qpos)] = home_qpos

    # Set initial control targets to match current position (no movement yet)
    # --- Base controls ---
    data.ctrl[base_ctrl_x] = 0.0
    data.ctrl[base_ctrl_y] = 0.0
    data.ctrl[base_ctrl_theta] = 0.0

    # --- Right arm controls (all at zero = home position) ---
    data.ctrl[right_arm_ctrl_waist] = 0.0
    data.ctrl[right_arm_ctrl_shoulder] = 0.0
    data.ctrl[right_arm_ctrl_elbow] = 0.0
    data.ctrl[right_arm_ctrl_forearm_roll] = 0.0
    data.ctrl[right_arm_ctrl_wrist_angle] = 0.0
    data.ctrl[right_arm_ctrl_wrist_rotate] = 0.0
    data.ctrl[right_gripper_ctrl_left_finger] = 0.015  # 0.015 = fully open
    data.ctrl[right_gripper_ctrl_right_finger] = 0.015  # 0.015 = fully open

    # --- Left arm controls (all at zero = home position) ---
    data.ctrl[left_arm_ctrl_waist] = 0.0
    data.ctrl[left_arm_ctrl_shoulder] = 0.0
    data.ctrl[left_arm_ctrl_elbow] = 0.0
    data.ctrl[left_arm_ctrl_forearm_roll] = 0.0
    data.ctrl[left_arm_ctrl_wrist_angle] = 0.0
    data.ctrl[left_arm_ctrl_wrist_rotate] = 0.0
    data.ctrl[left_gripper_ctrl_left_finger] = 0.015  # 0.015 = fully open
    data.ctrl[left_gripper_ctrl_right_finger] = 0.015  # 0.015 = fully open

    # --- Camera pan-tilt controls (start centered) ---
    data.ctrl[camera_pan_ctrl] = 0.0   # 0 = looking straight ahead
    data.ctrl[camera_tilt_ctrl] = 0.0  # 0 = level with horizon

    # Compute forward kinematics to update all derived quantities
    mujoco.mj_forward(model, data)

    # ==========================================================================
    # STEP 6: Set up the demo motion with speed-controlled targets
    # ==========================================================================
    print("=" * 60)
    print("TidyBot2 Bimanual WX250s Arms - Movement Demo")
    print("=" * 60)
    print(f"\nSpeed Settings:")
    print(f"  Linear speed:  {BASE_LINEAR_SPEED:.2f} m/s")
    print(f"  Angular speed: {BASE_ANGULAR_SPEED:.2f} rad/s")
    print(f"\nTarget Position:")
    print(f"  X: {TARGET_X_POSITION:.2f} m")
    print(f"  Y: {TARGET_Y_POSITION:.2f} m")
    print(f"  Theta: {TARGET_THETA:.2f} rad")
    print(f"\nCamera capture positions: {len(CAMERA_CAPTURE_POSITIONS)}")
    print(f"  Images will be saved to: {output_dir}")
    print(f"\nInitial position: x={data.qpos[base_joint_x]:.3f}m, "
          f"y={data.qpos[base_joint_y]:.3f}m, "
          f"theta={data.qpos[base_joint_theta]:.3f}rad")
    print("=" * 60)

    # -------------------------------------------------------------------------
    # Speed Control Variables
    # -------------------------------------------------------------------------
    # These track the CURRENT target position that we send to the actuator.
    # By incrementing these gradually toward the goal, we control the speed.
    current_target_x = 0.0      # Current X target (starts at origin)
    current_target_y = 0.0      # Current Y target (starts at origin)
    current_target_theta = 0.0  # Current theta target (starts at 0)

    # -------------------------------------------------------------------------
    # Camera Control Variables
    # -------------------------------------------------------------------------
    current_camera_pan = 0.0           # Current pan target (radians)
    current_camera_tilt = 0.0          # Current tilt target (radians)
    camera_position_index = 0          # Which capture position we're moving to
    camera_settled_time = None         # Time when camera reached target position
    camera_demo_started = False        # Has the camera demo phase begun?
    camera_demo_complete = False       # Has the camera demo finished?
    CAMERA_SETTLE_TIME = 0.5           # Seconds to wait before capturing (let servo settle)

    # ==========================================================================
    # STEP 7: Run the simulation with visualization
    # ==========================================================================
    # launch_passive creates a viewer that doesn't block - we control the sim loop
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Configure the camera for a good view of the robot
        viewer.cam.distance = 3.0      # Distance from target point (meters)
        viewer.cam.elevation = -20     # Angle above/below horizon (degrees)
        viewer.cam.azimuth = 90        # Rotation around vertical axis (degrees)

        # Disable transparency for clearer visualization
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False

        # Show coordinate frames (body frames and world frame)
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY

        # Track simulation state
        start_time = time.time()
        base_target_reached = False
        arm_demo_complete = False

        print(f"\nDriving forward to x={TARGET_X_POSITION}m while moving arms...")

        # ----------------------------------------------------------------------
        # Main simulation loop - runs until you close the viewer window
        # ----------------------------------------------------------------------
        while viewer.is_running():
            step_start = time.time()
            elapsed_time = time.time() - start_time

            # ==================================================================
            # CONTROL THE MOBILE BASE (Speed-Limited)
            # ==================================================================
            # Instead of jumping directly to the target, we increment the
            # current target position by (speed * dt) each timestep.
            # This gives us precise control over the movement speed.
            #
            # Formula: new_target = current_target + direction * speed * dt
            # Where direction is +1 or -1 depending on which way we need to go.
            # We use np.clip to ensure we don't overshoot the goal.

            # --- X Position (forward/backward) ---
            if current_target_x < TARGET_X_POSITION:
                # Moving forward: increment target
                current_target_x = min(
                    current_target_x + BASE_LINEAR_SPEED * dt,
                    TARGET_X_POSITION  # Don't overshoot
                )
            elif current_target_x > TARGET_X_POSITION:
                # Moving backward: decrement target
                current_target_x = max(
                    current_target_x - BASE_LINEAR_SPEED * dt,
                    TARGET_X_POSITION  # Don't overshoot
                )

            # --- Y Position (strafe left/right) ---
            if current_target_y < TARGET_Y_POSITION:
                current_target_y = min(
                    current_target_y + BASE_LINEAR_SPEED * dt,
                    TARGET_Y_POSITION
                )
            elif current_target_y > TARGET_Y_POSITION:
                current_target_y = max(
                    current_target_y - BASE_LINEAR_SPEED * dt,
                    TARGET_Y_POSITION
                )

            # --- Theta (rotation) ---
            if current_target_theta < TARGET_THETA:
                current_target_theta = min(
                    current_target_theta + BASE_ANGULAR_SPEED * dt,
                    TARGET_THETA
                )
            elif current_target_theta > TARGET_THETA:
                current_target_theta = max(
                    current_target_theta - BASE_ANGULAR_SPEED * dt,
                    TARGET_THETA
                )

            # Send the current (speed-limited) targets to the actuators
            data.ctrl[base_ctrl_x] = current_target_x
            data.ctrl[base_ctrl_y] = current_target_y
            data.ctrl[base_ctrl_theta] = current_target_theta

            # ==================================================================
            # CONTROL BOTH ARMS - Demo: Raise arms, wave, return home
            # ==================================================================
            # Arm joint values are in radians. Positive/negative depends on
            # the joint's axis definition in the URDF/XML.

            if elapsed_time < ARM_RAISE_DURATION:
                # Phase 1: Raise both arms into a bent position
                # Right arm
                data.ctrl[right_arm_ctrl_shoulder] = -0.5      # Raise shoulder (negative = up)
                data.ctrl[right_arm_ctrl_elbow] = 1.0          # Bend elbow
                data.ctrl[right_arm_ctrl_wrist_angle] = -0.5   # Angle wrist
                # Left arm (mirrored)
                data.ctrl[left_arm_ctrl_shoulder] = -0.5
                data.ctrl[left_arm_ctrl_elbow] = 1.0
                data.ctrl[left_arm_ctrl_wrist_angle] = -0.5

            elif elapsed_time < ARM_RAISE_DURATION + ARM_WAVE_DURATION:
                # Phase 2: Wave by oscillating the waist joints
                # np.sin creates smooth back-and-forth motion
                wave_angle = np.sin(elapsed_time * ARM_WAVE_FREQUENCY * 2 * np.pi) * ARM_WAVE_AMPLITUDE
                # Right arm
                data.ctrl[right_arm_ctrl_waist] = wave_angle
                data.ctrl[right_arm_ctrl_shoulder] = -0.5
                data.ctrl[right_arm_ctrl_elbow] = 1.0
                data.ctrl[right_arm_ctrl_wrist_angle] = -0.5
                # Left arm (wave in opposite phase for variety)
                data.ctrl[left_arm_ctrl_waist] = -wave_angle
                data.ctrl[left_arm_ctrl_shoulder] = -0.5
                data.ctrl[left_arm_ctrl_elbow] = 1.0
                data.ctrl[left_arm_ctrl_wrist_angle] = -0.5

            elif not arm_demo_complete:
                # Phase 3: Return all arm joints to home position
                # Right arm
                data.ctrl[right_arm_ctrl_waist] = 0.0
                data.ctrl[right_arm_ctrl_shoulder] = 0.0
                data.ctrl[right_arm_ctrl_elbow] = 0.0
                data.ctrl[right_arm_ctrl_wrist_angle] = 0.0
                # Left arm
                data.ctrl[left_arm_ctrl_waist] = 0.0
                data.ctrl[left_arm_ctrl_shoulder] = 0.0
                data.ctrl[left_arm_ctrl_elbow] = 0.0
                data.ctrl[left_arm_ctrl_wrist_angle] = 0.0
                arm_demo_complete = True
                print("Bimanual arm demo complete!")

            # ==================================================================
            # CONTROL THE PAN-TILT CAMERA
            # ==================================================================
            # After the arm demo completes, demonstrate camera control by moving
            # to each capture position and taking RGB + depth images.
            #
            # Camera control works the same as other actuators:
            #   data.ctrl[camera_pan_ctrl] = target_angle_in_radians
            #   data.ctrl[camera_tilt_ctrl] = target_angle_in_radians

            if arm_demo_complete and not camera_demo_complete:

                # Start camera demo on first frame after arm demo completes
                if not camera_demo_started:
                    camera_demo_started = True
                    print("\n" + "=" * 60)
                    print("Starting Camera Pan-Tilt Demo")
                    print("=" * 60)
                    target_pan, target_tilt = CAMERA_CAPTURE_POSITIONS[0]
                    print(f"\nMoving to position 1/{len(CAMERA_CAPTURE_POSITIONS)}: "
                          f"pan={np.degrees(target_pan):.1f}°, tilt={np.degrees(target_tilt):.1f}°")

                # Get current target position from the list
                target_pan, target_tilt = CAMERA_CAPTURE_POSITIONS[camera_position_index]

                # -------------------------------------------------------------
                # Smoothly move pan toward target (speed-limited)
                # -------------------------------------------------------------
                if current_camera_pan < target_pan:
                    current_camera_pan = min(
                        current_camera_pan + CAMERA_PAN_SPEED * dt,
                        target_pan
                    )
                elif current_camera_pan > target_pan:
                    current_camera_pan = max(
                        current_camera_pan - CAMERA_PAN_SPEED * dt,
                        target_pan
                    )

                # -------------------------------------------------------------
                # Smoothly move tilt toward target (speed-limited)
                # -------------------------------------------------------------
                if current_camera_tilt < target_tilt:
                    current_camera_tilt = min(
                        current_camera_tilt + CAMERA_TILT_SPEED * dt,
                        target_tilt
                    )
                elif current_camera_tilt > target_tilt:
                    current_camera_tilt = max(
                        current_camera_tilt - CAMERA_TILT_SPEED * dt,
                        target_tilt
                    )

                # -------------------------------------------------------------
                # Send control commands to camera actuators
                # -------------------------------------------------------------
                data.ctrl[camera_pan_ctrl] = current_camera_pan
                data.ctrl[camera_tilt_ctrl] = current_camera_tilt

                # -------------------------------------------------------------
                # Check if camera has reached target position
                # -------------------------------------------------------------
                pan_at_target = abs(current_camera_pan - target_pan) < 0.01
                tilt_at_target = abs(current_camera_tilt - target_tilt) < 0.01

                if pan_at_target and tilt_at_target:
                    # Camera reached target - start settle timer
                    if camera_settled_time is None:
                        camera_settled_time = time.time()

                    # Wait for camera to settle before capturing
                    if time.time() - camera_settled_time > CAMERA_SETTLE_TIME:
                        # ---------------------------------------------------------
                        # CAPTURE IMAGES FROM THE REALSENSE D435
                        # ---------------------------------------------------------
                        print(f"\n  Capturing images at position {camera_position_index + 1}...")

                        # Capture RGB image
                        rgb_image = capture_rgb_image(renderer, data, "d435_rgb")
                        rgb_filename = output_dir / f"rgb_pos{camera_position_index}_pan{target_pan:.2f}_tilt{target_tilt:.2f}.png"
                        save_image(rgb_image, rgb_filename)

                        # Capture depth image
                        depth_image = capture_depth_image(renderer, data, "d435_depth")
                        depth_filename = output_dir / f"depth_pos{camera_position_index}_pan{target_pan:.2f}_tilt{target_tilt:.2f}.png"
                        save_image(depth_image, depth_filename)

                        # Move to next position
                        camera_position_index += 1
                        camera_settled_time = None  # Reset settle timer

                        if camera_position_index >= len(CAMERA_CAPTURE_POSITIONS):
                            # All positions captured
                            camera_demo_complete = True
                            print("\n" + "=" * 60)
                            print("Camera demo complete!")
                            print(f"Images saved to: {output_dir}")
                            print("=" * 60)
                            print("\nClose the viewer window to exit.")
                        else:
                            # Announce next position
                            next_pan, next_tilt = CAMERA_CAPTURE_POSITIONS[camera_position_index]
                            print(f"\nMoving to position {camera_position_index + 1}/{len(CAMERA_CAPTURE_POSITIONS)}: "
                                  f"pan={np.degrees(next_pan):.1f}°, tilt={np.degrees(next_tilt):.1f}°")

            # ==================================================================
            # STEP THE SIMULATION
            # ==================================================================
            # This advances physics by one timestep (defined in model.opt.timestep)
            mujoco.mj_step(model, data)

            # Sync the viewer to show the updated state
            viewer.sync()

            # ==================================================================
            # CHECK IF BASE REACHED TARGET
            # ==================================================================
            current_x = data.qpos[base_joint_x]  # Read current x position
            if not base_target_reached and abs(current_x - TARGET_X_POSITION) < 0.01:
                travel_time = time.time() - start_time
                actual_speed = TARGET_X_POSITION / travel_time
                print(f"\nBase target reached!")
                print(f"  Final position: x={current_x:.3f}m")
                print(f"  Time taken: {travel_time:.2f}s")
                print(f"  Average speed: {actual_speed:.2f} m/s")
                print("\nClose the viewer window to exit.")
                base_target_reached = True

            # ==================================================================
            # MAINTAIN REAL-TIME SIMULATION SPEED
            # ==================================================================
            # Sleep to match real time (prevents simulation from running too fast)
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    print("Simulation complete!")


if __name__ == "__main__":
    main()
