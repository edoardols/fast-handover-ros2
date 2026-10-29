#!/usr/bin/env python3

import time
import copy
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
import tf2_ros
import tf_transformations as tf_trans
from handover_interfaces.action import Pose


class PoseActionServer:
    def __init__(self, node, frame_base, ee_frame):
        self.node = node
        self.frame_base = frame_base
        self.ee_frame = ee_frame
        self.cb_group = ReentrantCallbackGroup()

        # [LEGACY ROS 1]
        # rospy.wait_for_service('/compute_fk',timeout=500)
        # self.fk_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.sub_joint_states = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            1,
            callback_group=self.cb_group
        )
        self.sub_goal_pose = self.node.create_subscription(
            PoseStamped,
            '/grasp_pose',
            self.goal_pose_callback,
            1,
            callback_group=self.cb_group
        )
        self.sub_vel_cmd = self.node.create_subscription(
            TwistStamped,
            '/servo_server/delta_twist_cmds',
            self.velocity_callback,
            1,
            callback_group=self.cb_group
        )

        self.vel_pub = self.node.create_publisher(
            TwistStamped,
            '/servo_server/delta_twist_cmds',
            1
        )
        self.pose_pub = self.node.create_publisher(
            PoseStamped,
            '/current_pose',
            1
        )

        self.current_joint_state = None
        self.cur_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.vel_cmd = TwistStamped()

        # Gain PID
        self.Kp = np.array([1.9, 1.9, 2.3]) # 1.8 1.8 2.2
        self.Ki = np.array([0.0, 0.0, 0.0])
        self.Kd = np.array([0.05, 0.05, 0.05])

        self.Kp_ori = np.array([0.2, 0.2, 0.2])
        self.Ki_ori = np.array([0.1, 0.1, 0.1])
        self.Kd_ori = np.array([0.001, 0.001, 0.001])

        self.last_err = np.zeros(3)
        self.last_ori_err = np.zeros(3)

        self._action_server = ActionServer(
            self.node,
            Pose,
            'pose_action_server',
            execute_callback=self.execute_cb,
            callback_group=self.cb_group
        )
        self.node.get_logger().info("Started Pose ActionServer")

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.update_current_pose()

    def goal_pose_callback(self, msg):
        self.goal_pose = msg

    def velocity_callback(self, msg):
        self.vel_cmd = msg

    def update_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame_base, #'panda_link0'
                self.ee_frame, #'robotiq_virtual_link'
                rclpy.time.Time()
            )
            self.cur_pose.header = t.header
            self.cur_pose.pose.position.x = t.transform.translation.x
            self.cur_pose.pose.position.y = t.transform.translation.y
            self.cur_pose.pose.position.z = t.transform.translation.z
            self.cur_pose.pose.orientation = t.transform.rotation
            self.pose_pub.publish(self.cur_pose)
        except Exception as e:
            self.node.get_logger().info(f'Failed to perform FK: {e}')

    # [LEGACY ROS 1 no more needed since subscriber /grasp_pose]
    # def WaitForGoal(self):
    #     try:
    #         goal = rospy.wait_for_message("/grasp_pose", PoseStamped, timeout=60)  # Wait for 60 seconds for a point cloud
    #     except rospy.ROSException as e:
    #         rospy.logerr("Failed to receive goal pose: %s", e)
    #         return None
    #     return goal

    async def execute_cb(self, goal_handle):
        goal_msg = goal_handle.request

        q_ee = [self.cur_pose.pose.orientation.x, self.cur_pose.pose.orientation.y,
                self.cur_pose.pose.orientation.z, self.cur_pose.pose.orientation.w]
        R_EE = tf_trans.quaternion_matrix(q_ee)

        # Verifying if a goal is sent directly with the action call
        if goal_msg.goal_pose.pose.position.x != 0 or goal_msg.goal_pose.pose.position.y != 0 or goal_msg.goal_pose.pose.position.z != 0:
            self.node.get_logger().info('Received goal directly from action call.')
            self.goal_pose = goal_msg.goal_pose

            if goal_msg.flag:
                time.sleep(0.5)
                offset_world = R_EE.dot([0, 0, goal_msg.goal_pose.pose.position.z, 0])
                self.goal_pose = copy.deepcopy(self.cur_pose)
                self.node.get_logger().info(f'Offset world: {offset_world}')
                self.goal_pose.pose.position.x += offset_world[0]
                self.goal_pose.pose.position.y += offset_world[1]
                self.goal_pose.pose.position.z += offset_world[2]
        else:
            self.node.get_logger().info('Waiting for goal from /grasp_pose topic.')
            # [LEGACY ROS 1]
            # if self.goal_pose is None:
            #     self.a_server.set_aborted()
            #     rospy.logerr("No goal received, action aborted.")
            #     return

        success = self.control_loop(goal_handle)

        result = Pose.Result()
        if success:
            result.pose = self.cur_pose
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def control_loop(self, goal_handle):
        dt = 1/50.0 # OLD: 50
        position_integral = np.zeros(3)
        orientation_integral = np.zeros(3)
        max_velocity = 0.06 # Start with a lower max velocity
        velocity_ramp_rate = 0.025 # Incremental increase per cycle

        velocity_command_filt = np.array([
            self.vel_cmd.twist.linear.x,
            self.vel_cmd.twist.linear.y,
            self.vel_cmd.twist.linear.z
        ])
        alpha = 0.5

        start_time = time.time()
        while time.time() - start_time < 20.0:
            if goal_handle.is_cancel_requested:
                self.node.get_logger().info('Action preempted')
                return False

            current_position = np.array([self.cur_pose.pose.position.x,
                                         self.cur_pose.pose.position.y,
                                         self.cur_pose.pose.position.z])
            goal_position = np.array([self.goal_pose.pose.position.x,
                                      self.goal_pose.pose.position.y,
                                      self.goal_pose.pose.position.z])

            position_error = goal_position - current_position
            position_integral += position_error * dt # Update integral error
            position_error_derivative = (position_error - self.last_err) / dt
            self.last_err = position_error

            # Calculate velocity command with PID
            raw_velocity_command = (self.Kp * position_error +
                                    self.Kd * position_error_derivative +
                                    self.Ki * position_integral)

            # Limit the velocity to prevent high initial jerk
            velocity_command = np.clip(raw_velocity_command, -max_velocity, max_velocity)
            # Gradually increase the max velocity limit
            max_velocity = min(max_velocity + velocity_ramp_rate, 1.0)

            current_orientation = np.array([self.cur_pose.pose.orientation.x,
                                            self.cur_pose.pose.orientation.y,
                                            self.cur_pose.pose.orientation.z,
                                            self.cur_pose.pose.orientation.w])
            goal_orientation = np.array([self.goal_pose.pose.orientation.x,
                                         self.goal_pose.pose.orientation.y,
                                         self.goal_pose.pose.orientation.z,
                                         self.goal_pose.pose.orientation.w])

            ori_error = tf_trans.euler_from_quaternion(
                tf_trans.quaternion_multiply(goal_orientation,
                                             tf_trans.quaternion_inverse(current_orientation)))
            ori_error_derivative = np.array(ori_error) - np.array(self.last_ori_err)
            self.last_ori_err = ori_error

            angular_velocity_command = (self.Kp_ori * ori_error +
                                        self.Kd_ori * ori_error_derivative +
                                        self.Ki_ori * orientation_integral).tolist()

            velocity_command_filt += alpha * (velocity_command - velocity_command_filt)

            twist = TwistStamped()
            twist.header.stamp = self.node.get_clock().now().to_msg()
            twist.header.frame_id = self.frame_base
            twist.twist.linear.x = float(velocity_command_filt[0])
            twist.twist.linear.y = float(velocity_command_filt[1])
            twist.twist.linear.z = float(velocity_command_filt[2])

            # twist.twist.angular.x = float(angular_velocity_command[0])
            # twist.twist.angular.y = float(angular_velocity_command[1])
            # twist.twist.angular.z = float(angular_velocity_command[2])
            
            twist.twist.angular.x = 0.0
            twist.twist.angular.y = 0.0
            twist.twist.angular.z = 0.0

            self.vel_pub.publish(twist)

            if np.linalg.norm(position_error) < 0.03: # Check if the goal is reached
                self.node.get_logger().info('Goal reached')
                return True

            # [LEGACY ROS 1]
            # rate = rospy.Rate(50)  # OLD: 50
            time.sleep(dt)

        return False