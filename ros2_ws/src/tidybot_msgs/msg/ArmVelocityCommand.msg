# Joint velocity command for a 6-DOF WX250s arm
# Joint order: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate

std_msgs/Header header

# Target velocities in rad/s for each joint
float64[6] joint_velocities

# Duration to apply velocity in seconds (0 = until next command)
float64 duration
