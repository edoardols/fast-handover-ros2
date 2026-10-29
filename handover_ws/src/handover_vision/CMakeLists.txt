cmake_minimum_required(VERSION 3.8)
project(handover_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  handover_vision/handover_vision/hand_detector.py
  handover_vision/handover_vision/kalman_filter_node.py
  handover_vision/handover_vision/object_detector.py
  handover_vision/handover_vision/pointcloud_merger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
