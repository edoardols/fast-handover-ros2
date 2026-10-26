## Overview

The `handover_action` ROS package implements a set of custom `actionlib` actions for managing a **vision-based human-to-robot handover pipeline**, utilizing a **state machine architecture** (SMACH) for control. 
The package provides actions that control various components involved in the handover sequence, including robot motion, gripper control, and idle/rest states. 

## Actions

Below is a list of action definitions provided in the `action/` directory, each handling a specific task or sequence in the handover pipeline:

1. `Gripper.action`
This action controls the opening or closing of the gripper to facilitate object grasping and placement.

- **Usage:** Executed during the `GRASP` and `PLACE` states of the pipeline.
- **Goal:**
  - `std_msgs/Float64 cmd_gripper` - Command for opening or closing the gripper.
- **Result:**
  - `geometry_msgs/PoseStamped current_pose` - The final pose of the robot's end effector after the gripper action.

2. `Joint.action`
This action specifies joint configuration goals for the robot arm, ensuring the robot reaches specific poses or enters predefined states (such as rest or home positions).

- **Usage:** Used to move the robot arm to specific joint configurations, often for setting up the arm before or after a handover.
- **Goal:**
  - `std_msgs/Float64Multiarray joint` - Joint configuration values (not used in this pipeline).
  - `std_msgs/Bool rest` - Set to `True` during the `REST` state and `False` during the `MOVE_TO_HOME` state.
- **Result:**
  - `std_msgs/Bool res` - Outcome of the joint action.

3. `Pose.action`
This action defines a pose goal for the robot’s end effector.

- **Usage:** Used during the `HAND_TRACKING` state to track the object’s initial position and during the `FINAL_POSE` state to define where the object should be placed.
- **Goal:**
  - `geometry_msgs/PoseStamped goal` - The target pose for the object or the robot’s end effector.
  - `std_msgs/Bool flag` - Set to `True` during the `GO_BACK` state to apply an offset transformation in the world reference frame.
- **Result:**
  - `geometry_msgs/PoseStamped pose` - The final pose of the robot’s end effector after execution.
