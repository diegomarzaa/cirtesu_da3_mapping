cmake_minimum_required(VERSION 3.8)
project(cirtesu_da3_mapping)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python module (cirtesu_da3_mapping/)
ament_python_install_package(${PROJECT_NAME})

# Install executable scripts
install(PROGRAMS
  scripts/ply_publisher_node.py
  scripts/camera_poses_publisher_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, rviz, config directories
install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
