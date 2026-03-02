cmake_minimum_required(VERSION 3.5)
project(second_challenge_Kusunose)

find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(second_challenge_Kusunose_node
  src/second_challenge.cpp
  src/second_challenge_node.cpp
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()