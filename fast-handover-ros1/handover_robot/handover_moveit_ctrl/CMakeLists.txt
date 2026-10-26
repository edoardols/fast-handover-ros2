cmake_minimum_required(VERSION 2.8.3)
project(handover_moveit_ctrl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED 
    roscpp
    rospy
    std_msgs
     moveit_core
  moveit_ros_planning_interface
)

catkin_package(
CATKIN_DEPENDS std_msgs 
)

include_directories(
  #include 
  ${catkin_INCLUDE_DIRS}
)