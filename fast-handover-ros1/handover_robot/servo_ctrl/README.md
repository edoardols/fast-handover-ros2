## Overview

The `servo_ctrl` ROS package integrates with **MoveIt Servo** to allow real-time motion control of the robotic arm. It includes configuration files for two robotic platforms: the **UR5** and **Panda** robots. The package allows control using **cartesian velocity control** commands and **joint space control**.


## Configuration Files

The package includes two configuration files, `config_ur.yaml` and `config_panda.yaml`, for both **UR5** and **Panda** robots.

## Launch File

The `servo.launch` file can be used to start the servoing system for either robot.


To launch for **UR5**:

```bash
roslaunch handover_moveit_ctrl servo.launch ur5:=true
```

To launch for **Panda**:

```bash
roslaunch handover_moveit_ctrl servo.launch panda:=true
```
