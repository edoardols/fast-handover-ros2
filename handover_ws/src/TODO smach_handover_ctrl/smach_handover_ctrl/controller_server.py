#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from smach_handover_ctrl.joint_action_server import JointActionServer
from smach_handover_ctrl.pose_action_server import PoseActionServer
from smach_handover_ctrl.camera_action_server import CameraActionServer
from smach_handover_ctrl.gripper_action_server import GripperActionServer

# [LEGACY ROS 1]
# import rosgraph
# import rospy
# if not rosgraph.is_master_online():
#     print('Error: ROS master not running')
#     exit(1)


def main(args=None):
    rclpy.init(args=args)
    node = Node("handover_action_server")

    node.declare_parameter("robot", "ur5")
    robot = node.get_parameter("robot").get_parameter_value().string_value
    node.get_logger().info(f"Arm: {robot}")

    if robot == "ur5":
        frame_base = "base_link"
        ee_frame = "robotiq_virtual_link"
        # [LEGACY ROS 1] controller1 = "vel_joint_traj_controller"
        controller1 = "ur5_arm_controller"
        movegroup = "manipulator"
        joint_config = [-0.07, -2.23, 2.32, -0.087, 1.57, 3.14] # initial config ur5
    elif robot == "panda":
        frame_base = "panda_link0"
        ee_frame = "panda_hand_tcp"
        # [LEGACY ROS 1] controller1 = "effort_joint_trajectory_controller"
        controller1 = "panda_arm_controller"
        movegroup = "panda_manipulator"
        # joint_config = [0.0, -0.5, 0.06, -2.45, 0.0, 2.0, 0.0] # initial config panda
        joint_config = [0.0, -0.87, 0.028, -2.86, 0.0, 1.6, 0.0] # initial config panda
    else:
        node.get_logger().error(f"Robotic arm not supported: {robot}")
        sys.exit(1)

    # Istanziazione modulare dei 4 Action Server
    joint_act_srv = JointActionServer(node=node, joint_config=joint_config, controller1=controller1, movegroup=movegroup)
    pose_act_srv = PoseActionServer(node=node, frame_base=frame_base, ee_frame=ee_frame)
    gripper_act_srv = GripperActionServer(node=node, arm=robot)
    camera_act_srv = CameraActionServer(node=node, arm=robot, frame_base=frame_base, ee_frame=ee_frame)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()