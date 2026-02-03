cmake_minimum_required(VERSION 3.8)
project(tidybot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install rviz config files
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/test_base_sim.py
  scripts/test_camera_sim.py
  scripts/test_arms_sim.py
  scripts/test_planner_sim.py
  scripts/test_real_hardware.py
  scripts/test_base_real.py
  scripts/test_arms_real.py
  scripts/test_grippers_real.py
  scripts/test_pan_tilt_real.py
  scripts/test_torque_hold.py
  scripts/test_full_robot.py
  scripts/test_tf.py
  scripts/test_planner_real.py
  scripts/diagnose_dynamixel.py
  scripts/example_state_machine.py
  scripts/trajectory_tracking.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
