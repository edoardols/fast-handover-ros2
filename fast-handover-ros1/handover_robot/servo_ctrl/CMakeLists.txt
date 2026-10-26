cmake_minimum_required(VERSION 2.8.3)
project(servo_ctrl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_servo
)
add_definitions(-std=c++11)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES soma_ur5
  CATKIN_DEPENDS moveit_servo #roscpp rospy tf2 ur_modern_driver actionlib_msgs message_runtime
  #DEPENDS system_lib
)

# include_directories(include)
include_directories(  ${catkin_INCLUDE_DIRS}   include )
link_directories( ../../devel/lib lib)
include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

