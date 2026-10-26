cmake_minimum_required(VERSION 3.0.2)
project(handover_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  vision_msgs
  geometry_msgs
  message_generation
  trajectory_msgs
  franka_msgs
)

add_message_files(
  FILES
  Keypoints2D.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  vision_msgs
  geometry_msgs
  trajectory_msgs
  franka_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime vision_msgs geometry_msgs franka_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)