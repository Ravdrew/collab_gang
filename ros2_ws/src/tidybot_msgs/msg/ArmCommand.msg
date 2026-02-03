# Target joint positions for a 6-DOF WX250s arm
# Joint order: waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate

std_msgs/Header header

# Target positions in radians for each joint
float64[6] joint_positions

# Time to reach target in seconds (0 = immediate/as fast as possible)
float64 duration
