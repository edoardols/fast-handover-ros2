cmake_minimum_required(VERSION 2.8.3)
project(handover_action)

find_package(catkin REQUIRED COMPONENTS 
            actionlib_msgs
            geometry_msgs
            sensor_msgs
            std_msgs
            message_generation)
            
## Generate actions in the 'action' folder ## DIRECTORY action
add_action_files(
                 FILES
  Pose.action
  Joint.action
  Gripper.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs  # Or other packages containing msgs
  geometry_msgs
  std_msgs
)

catkin_package( CATKIN_DEPENDS actionlib_msgs std_msgs geometry_msgs message_runtime)
