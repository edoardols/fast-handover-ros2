## Overview

The `handover_moveit_ctrl` provides control functionalities for a vision-based **human-to-robot handover pipeline**. It uses **MoveIt!** for motion planning and execution, enabling robotic arms to interact with humans in collaborative tasks.


## Launch Files
This package includes launch files to configure MoveIt! for **UR5** and **Panda** robots:

   
1. `ur5.launch`  
   Launches the MoveIt! configuration for the **UR5** robot.

```bash
roslaunch handover_moveit_ctrl ur5.launch robot_ip:=<robot_ip>

```

2. `panda.launch`  
   Launches the MoveIt! configuration for the **Panda** robot.

```bash
roslaunch handover_moveit_ctrl panda.launch robot_ip:=<robot_ip>
```
