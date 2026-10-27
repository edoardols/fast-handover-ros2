#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from handover_interfaces.action import Gripper


class GripperActionServer:
    def __init__(self, node, arm):
        self.node = node
        self.arm = arm
        self.cb_group = ReentrantCallbackGroup()
        self.cur_pose = PoseStamped()

        # Publisher diretto
        if self.arm == "panda":
            self.gripper_pub = self.node.create_publisher(
                Float64,
                '/franka_gripper/main_cmd',
                10
            )
        else:
            self.gripper_pub = self.node.create_publisher(
                Float64,
                '/robotiq_2f_85_gripper/command',
                10
            )

        self.sub_cur_pose = self.node.create_subscription(
            PoseStamped,
            '/current_pose',
            self.cur_pose_callback,
            1,
            callback_group=self.cb_group
        )

        self._action_server = ActionServer(
            self.node,
            Gripper,
            'gripper_action_server',
            execute_callback=self.execute_cb,
            callback_group=self.cb_group
        )
        self.node.get_logger().info("Started Gripper ActionServer (1:1 Porting)")

    def cur_pose_callback(self, msg):
        self.cur_pose = msg

    def open_gripper(self, width=0.0):
        cmd = Float64()
        cmd.data = float(width) if self.arm == "panda" else 0.0
        self.gripper_pub.publish(cmd)

    def close_gripper(self, width):
        cmd = Float64()
        cmd.data = float(width)
        self.gripper_pub.publish(cmd)

    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        result = Gripper.Result()

        self.node.get_logger().info(f"Gripper Command: {goal.cmd_gripper} on Robot: {self.arm}")

        if self.arm == "ur5":
            if goal.cmd_gripper > 0.0:
                self.close_gripper(goal.cmd_gripper)
            else:
                self.open_gripper(0.0)
            time.sleep(0.5)
        elif self.arm == "panda":
            if goal.flag:
                self.open_gripper(goal.cmd_gripper)
            else:
                self.close_gripper(goal.cmd_gripper)
            time.sleep(0.5)

        result.pose = self.cur_pose
        goal_handle.succeed()
        return result