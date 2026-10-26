## Running the State Machine

To run the state machine, you need to launch two nodes:

1. **`controller_server.py`**: Starts the action servers for joint control, pose control, and gripper control.
2. **`planner.py`**: Starts the SMACH state machine and coordinates the handover process.

To launch these nodes, use the following commands depending on the robot model:

### For **UR5** robot:

**In the first terminal:**

```bash
rosrun smach_handover_ctrl controller_server.py _robot:=ur5
```

**In the second terminal:**

```bash
rosrun smach_handover_ctrl planner.py _robot:=ur5
```

### For **Panda** robot:

**In the first terminal:**

```bash
rosrun smach_handover_ctrl controller_server.py _robot:=panda
```

**In the second terminal:**

```bash
rosrun smach_handover_ctrl planner.py _robot:=panda
```

Once the nodes are running, the robot will start the handover process, transitioning through various states like tracking, grasping, and placing objects based on the state machine's flow.

<p align="center">
      <img src="docs/smach.png" />
</p>
