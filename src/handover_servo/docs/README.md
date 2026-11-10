## Overview

The `servo_ctrl` ROS package integrates with **MoveIt Servo** to allow real-time motion control of the robotic arm. It includes configuration files for two robotic platforms: the **UR5** and **Panda** robots. The package allows control using **cartesian velocity control** commands and **joint space control**.


## Configuration Files

The package includes two configuration files, `config_ur5.yaml` and `config_panda.yaml`, for both **UR5** and **Panda** robots.

## Launch File

The `servo.launch.py` file can be used to start the servoing system for either robot.


To launch for **UR5**:

```bash
ros2 launch servo_ctrl servo.launch.py robot:=ur5
```

To launch for **Panda**:

```bash
ros2 launch servo_ctrl servo.launch.py robot:=panda
```
