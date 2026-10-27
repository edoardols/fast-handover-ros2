cmake_minimum_required(VERSION 3.8)
project(handover_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Installazione del modulo Python interno (handover_vision/)
ament_python_install_package(${PROJECT_NAME})

# Installazione dei nodi eseguibili Python
install(PROGRAMS
  nodes/hand_detector.py
  nodes/object_detector.py
  nodes/kalman_filter_node.py
  nodes/pointcloud_merger.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installazione delle cartelle di supporto (config, launch, rviz)
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()