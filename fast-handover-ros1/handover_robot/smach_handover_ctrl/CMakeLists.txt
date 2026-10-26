cmake_minimum_required(VERSION 2.8.3)
project(smach_handover_ctrl)

find_package(catkin REQUIRED COMPONENTS 
            actionlib_msgs
            geometry_msgs
            std_msgs
            sensor_msgs
            #message_generation
            handover_action)
            
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs  # Or other packages containing msgs
  geometry_msgs
  std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)