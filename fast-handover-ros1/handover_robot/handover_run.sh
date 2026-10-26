#!/bin/bash

# Open Terminator with ROSLayout
terminator -l ROSLayout &

# Ensure Terminator is open
sleep 2

# Start the Docker container in the 1st terminal
xdotool type "docker run -it --privileged -v ~/workspace:/home/workspace/shared --net=host --cap-add SYS_NICE --env='QT_X11_NO_MITSHM=1' --volume='/tmp/.X11-unix:/tmp/.X11-unix:rw' -e DISPLAY=unix${DISPLAY} --device /dev/dri/card0:/dev/dri/card0 -id ros_noetic:handover_robot" 

xdotool key Return

# Wait until the container has started
sleep 3

# Get the container's name
CONTAINER_ID=$(docker ps -lq)

xdotool type "docker exec -it $CONTAINER_ID bash"
xdotool key Return
xdotool type "roslaunch handover_moveit_ctrl ur5.launch robot_ip:=<robot_ip>"
xdotool key Return

#Go to the 2nd terminal
xdotool key Ctrl+Tab
xdotool key Return
xdotool type "docker exec -it $CONTAINER_ID bash"
xdotool key Return
xdotool type "rosrun smach_handover_ctrl controller_server.py _robot:=ur5"

#Go to the 3rd terminal
xdotool key Ctrl+Tab
xdotool key Return
xdotool type "docker exec -it $CONTAINER_ID bash"
xdotool key Return
xdotool type "roslaunch servo_ctrl servo.launch ur5:=true"
xdotool key Return
   
#Go to the 4th terminal
xdotool key Ctrl+Tab
xdotool key Return

xdotool type "sudo chmod 777 /dev/ttyUSB0"
xdotool key Return
xdotool type "ENTER_YOUR_PASSWORD"
xdotool key Return
xdotool type "docker exec -it $CONTAINER_ID bash"
xdotool key Return
xdotool type "rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0"
xdotool key Return

#Go to the 5th terminal
xdotool key Ctrl+Tab
xdotool key Return
xdotool type "docker exec -it $CONTAINER_ID bash"
xdotool key Return
xdotool type "rosrun smach_handover_ctrl planner.py _robot:=panda"
    

#Go to the 6th terminal
xdotool key Ctrl+Tab
xdotool key Return
xdotool type "docker exec -it $CONTAINER_ID bash"
xdotool key Return
xdotool type "rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py"
xdotool key Return

#Go to the 7th terminal
xdotool key Ctrl+Tab
xdotool key Return
xdotool type "docker exec -it $CONTAINER_ID bash"
xdotool key Return
xdotool type "cd /path/to/your/ros_workspace/src"
xdotool key Return
xdotool type "./kill_all.sh"

#Go to the 1st terminal
xdotool key Ctrl+Tab
xdotool key Return