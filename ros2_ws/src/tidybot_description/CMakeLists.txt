cmake_minimum_required(VERSION 3.8)
project(tidybot_description)

find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install mesh files (resolve symlink to copy actual files)
# meshes/ is a symlink to simulation/assets/meshes
get_filename_component(MESHES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/meshes" REALPATH)
install(DIRECTORY ${MESHES_DIR}/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
