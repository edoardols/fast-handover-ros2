#!/usr/bin/env python3

import copy
import time
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState, PointCloud2
from vision_msgs.msg import BoundingBox3D
import tf2_ros
import tf_transformations as tf_trans
from handover_interfaces.action import Pose

class CameraActionServer:
    def __init__(self, node, arm, frame_base, ee_frame):
        self.node = node
        self.arm = arm
        self.frame_base = frame_base
        self.ee_frame = ee_frame
        self.cb_group = ReentrantCallbackGroup()

        # [LEGACY ROS 1]
        # rospy.wait_for_service('/compute_fk')
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
        self.sub_hand_pose = self.node.create_subscription(
            PoseStamped,
            '/hand_pose',
            self.goal_pose_callback,
            1,
            callback_group=self.cb_group
        )

        # [LEGACY ROS 1]
        # def objPose(self):
        #     try:
        #         obj_pose = rospy.wait_for_message("/object_pose", PoseStamped, timeout=50)  # Wait for 5 seconds for an object pose
        #         offset = PoseStamped()
        #         offset.pose.position.x = self.goal_pose.pose.position.x - obj_pose.pose.position.x
        #         offset.pose.position.y = self.goal_pose.pose.position.y - obj_pose.pose.position.y
        #         offset.pose.position.z = self.goal_pose.pose.position.z - obj_pose.pose.position.z

        #         return obj_pose, offset # Directly return the received pose
        #     except rospy.ROSException as e:
        #         rospy.logerr("Failed to receive object pose: %s", e)
        #         return None
        #     return obj_pose, offset
        self.sub_obj_pose = self.node.create_subscription(
            PoseStamped,
            '/object_pose',
            self.object_pose_callback,
            1,
            callback_group=self.cb_group
        )

        # [LEGACY ROS 1]
        # def objBB(self):
        #     try:
        #         obj_bb = rospy.wait_for_message("/object_bb", BoundingBox3D, timeout=50)  # Wait for 5 seconds for an object pose
        #         offset = PoseStamped()
        #         offset.pose.position.x = obj_bb.center.position.x - self.goal_pose.pose.position.x
        #         offset.pose.position.y = obj_bb.center.position.y - self.goal_pose.pose.position.y 
        #         offset.pose.position.z = obj_bb.center.position.z - self.goal_pose.pose.position.z
    
        #         size = np.array([obj_bb.size.x,obj_bb.size.y,obj_bb.size.z])
    
        #         return obj_bb, offset, size # Directly return the received pose
        #     except rospy.ROSException as e:
        #         rospy.logerr("Failed to receive object pose: %s", e)
        #         return None
        #     return obj_bb, offset, size

        self.sub_obj_bb = self.node.create_subscription(
            BoundingBox3D,
            '/object_bb',
            self.obj_bb_callback,
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

        self.cur_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.object_pose = PoseStamped()
        self.obj_bb = BoundingBox3D()

        self.init_obj_bb = None
        self.offset = None
        self.size = None
        self.init_hand_pose = None
        self.optimal_z_offset = None
        self.rotated = False

        if self.arm == "ur5":
            self.Kp = np.array([1.8, 1.8, 1.8])
            self.Ki = np.array([0.0, 0.0, 0.0])
            self.Kd = np.array([0.06, 0.06, 0.06])
        elif self.arm == "panda":
            self.Kp = np.array([1.8, 1.8, 1.8])
            self.Ki = np.array([0.0, 0.0, 0.0])
            self.Kd = np.array([0.06, 0.06, 0.06])

        self.Kp_ori = np.array([1.5, 1.5, 1.5])
        self.Ki_ori = np.array([0.0, 0.0, 0.0])
        self.Kd_ori = np.array([0.1, 0.1, 0.1])

        self.last_err = np.zeros(3)
        self.last_ori_err = np.zeros(3)

        self._action_server = ActionServer(
            self.node,
            Pose,
            'camera_action_server',
            execute_callback=self.execute_cb,
            callback_group=self.cb_group
        )
        self.node.get_logger().info("Started Camera ActionServer")

    # [LEGACY ROS 1]
    # def ptCloud(self):
    #         try:
    #             obj = rospy.wait_for_message("/object_pointcloud", PointCloud2, timeout=500)  # Wait for 10 seconds for a point cloud
    #         except rospy.ROSException as e:
    #             rospy.logerr("Failed to receive point cloud: %s", e)
    #             return None
    #         return obj

    def joint_state_callback(self, msg):
        self.update_current_pose()

    def goal_pose_callback(self, msg):
        self.goal_pose = msg

    def object_pose_callback(self, msg):
        self.object_pose = msg

    def obj_bb_callback(self, msg):
        self.obj_bb = msg

    def update_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame_base,
                self.ee_frame,
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

    def calculate_z_offset(self, object_height, obj_z, hand_z):
        if hand_z > obj_z:
            return -0.35 * object_height #0.46
        else:
            return 0.4 * object_height

    def rotate_gripper(self, rotation, axis, goal_handle):
        dt = 1/50.0
        max_angular_velocity = 0.1 # Start with a lower max angular velocity
        angular_velocity_ramp_rate = 0.07 # Incremental increase per cycle

        angular_command_filt = np.zeros(3)
        alpha = 0.5

        orientation_integral = 0
        Kp = np.array([2.0, 2.0, 2.0])
        Ki = np.array([0.0, 0.0, 0.0])
        Kd = np.array([0.05, 0.05, 0.05])

        current_orientation = np.array([
            self.cur_pose.pose.orientation.x,
            self.cur_pose.pose.orientation.y,
            self.cur_pose.pose.orientation.z,
            self.cur_pose.pose.orientation.w
        ])
        # current_orientation * additional_rotation
        target_quaternion = tf_trans.quaternion_multiply(current_orientation, rotation)

        start_time = time.time()
        while time.time() - start_time < 5.0:
            if goal_handle.is_cancel_requested:
                self.node.get_logger().info('Action preempted')
                return False

            current_orientation = np.array([
                self.cur_pose.pose.orientation.x,
                self.cur_pose.pose.orientation.y,
                self.cur_pose.pose.orientation.z,
                self.cur_pose.pose.orientation.w
            ])

            ori_error = np.array(tf_trans.euler_from_quaternion(
                tf_trans.quaternion_multiply(target_quaternion,
                                             tf_trans.quaternion_inverse(current_orientation))))

            orientation_integral += ori_error * dt
            ori_error_derivative = (ori_error - self.last_ori_err) / dt
            self.last_ori_err = ori_error

            raw_angular_velocity = (Kp * ori_error +
                                    Kd * ori_error_derivative +
                                    Ki * orientation_integral)
            
            # print('raw',raw_angular_velocity_x)
            angular_velocity = np.clip(raw_angular_velocity, -max_angular_velocity, max_angular_velocity)
            max_angular_velocity = min(max_angular_velocity + angular_velocity_ramp_rate, 1.5)

            angular_command_filt += alpha * (angular_velocity - angular_command_filt)

            twist = TwistStamped()
            twist.header.stamp = self.node.get_clock().now().to_msg()
            twist.header.frame_id = self.frame_base
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0

            if axis == 'x':
                twist.twist.angular.x = float(angular_command_filt[0])
                twist.twist.angular.y = 0.0 # angular_command_filt[1]
                twist.twist.angular.z = 0.0 # angular_command_filt[2]

                if np.linalg.norm(ori_error[0]) < 0.01:
                    self.node.get_logger().info('EE rotated on X!')
                    return True
                
            elif axis == 'y':
                twist.twist.angular.x = 0.0 # angular_command_filt[0]
                twist.twist.angular.y = float(angular_command_filt[1])
                twist.twist.angular.z = 0.0 # angular_command_filt[2]

                if np.linalg.norm(ori_error[1]) < 0.01:
                    self.node.get_logger().info('EE rotated on Y!')
                    return True

            elif axis == 'y':
                twist.twist.angular.x = 0.0 # angular_command_filt[0]
                twist.twist.angular.y = 0.0 # angular_command_filt[1]
                twist.twist.angular.z = float(angular_command_filt[2])
                
                if np.linalg.norm(ori_error[2]) < 0.01:
                    self.node.get_logger().info('EE rotated on >!')
                    return True

            self.vel_pub.publish(twist)
            '''
            if np.linalg.norm(ori_error[0])<0.01 : 
                rospy.loginfo('EE rotated!')
                return True
            '''
            time.sleep(dt)

        return False

    async def execute_cb(self, goal_handle):
        if self.init_obj_bb is None:
            # Get the initial object pose
            self.init_obj_bb = self.obj_bb
            self.size = np.array([self.obj_bb.size.x, self.obj_bb.size.y, self.obj_bb.size.z])
            # rospy.logerr("No initial object bb received")

        # If the object is short and large the gripper is rotated 
        '''
        if self.size[1] > 0.09 and self.size[2]<0.09: #self.obj_size.height:
            if self.rotated == False:
                if self.arm == 'ur5': 
                    self.rotated = self.rotate_gripper(rotation=np.array([0,0, 0.70710678, 0.70710678]),axis='x')
                elif self.arm == 'panda':    
                    self.rotated = self.rotate_gripper(rotation=np.array([0,0, -0.70710678, 0.70710678]),axis='x')
                    rospy.sleep(2)
                    self.rotate_gripper(rotation=np.array([0,0, -0.5, 0.866025]),axis='y')
        '''

        # Check if the hand is close to the object  
        # while (np.abs(self.goal_pose.pose.position.x - self.init_obj_bb.center.position.x) > self.size[0]/2 + 0.05  or
        #         np.abs(self.goal_pose.pose.position.y - self.init_obj_bb.center.position.y) > self.size[1]/2 + 0.05  or
        #         np.abs(self.goal_pose.pose.position.z - 0.035 - self.init_obj_bb.center.position.z) > self.size[2]/2 + 0.05 or
        #         np.linalg.norm(np.array([self.goal_pose.pose.position.x,
        #                                 self.goal_pose.pose.position.y,
        #                                 self.goal_pose.pose.position.z - 0.035]) -
        #                         np.array([self.init_obj_bb.center.position.x,
        #                                 self.init_obj_bb.center.position.y,
        #                                 self.init_obj_bb.center.position.z]))) > 0.035+ np.linalg.norm(self.size)/2  and \
        #         self.init_hand_pose is None:
        #         tmp = None
            
        if self.init_hand_pose is None:
            self.offset = PoseStamped()
            self.init_hand_pose = copy.deepcopy(self.goal_pose)
            self.offset.pose.position.x = self.init_obj_bb.center.position.x - self.init_hand_pose.pose.position.x
            self.offset.pose.position.y = self.init_obj_bb.center.position.y - self.init_hand_pose.pose.position.y
            self.offset.pose.position.z = self.init_obj_bb.center.position.z - self.init_hand_pose.pose.position.z

            self.optimal_z_offset = self.calculate_z_offset(
                # self.size[2]
                self.size[2] if self.size is not None and len(self.size) > 2 else 0.1,
                self.obj_bb.center.position.z,
                self.init_hand_pose.pose.position.z - 0.035
            )
            if not self.rotated:
                self.offset.pose.position.z += self.optimal_z_offset

        success = False
        init_hand_z = self.init_hand_pose.pose.position.z

        # Start hand tracking
        if self.goal_pose.pose.position.z > init_hand_z + 0.02:
            success = self.control_loop(goal_handle) # goal.target_pose
            print('START HANDOVER')

        init_obj_pose = PoseStamped()
        init_obj_pose.header.stamp = self.node.get_clock().now().to_msg()
        init_obj_pose.header.frame_id = self.frame_base
        init_obj_pose.pose.position.x = self.init_obj_bb.center.position.x
        init_obj_pose.pose.position.y = self.init_obj_bb.center.position.y
        init_obj_pose.pose.position.z = self.init_obj_bb.center.position.z

        if self.rotated:
            init_obj_pose.pose.position.z += 0.04
        else:
            init_obj_pose.pose.position.z += 0.7 * (self.optimal_z_offset if self.optimal_z_offset else 0.05)

        result = Pose.Result()
        if success:
            result.pose = init_obj_pose
            self.init_obj_bb = None
            self.init_hand_pose = None
            self.size = None
            self.rotated = False
            self.offset = None
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def control_loop(self, goal_handle):
        dt = 1/50.0 # OLD: 50

        position_integral = 0
        orientation_integral = 0
        max_velocity = 0.15
        velocity_ramp_rate = 0.025

        max_angular_velocity = 0.03
        angular_velocity_ramp_rate = 0.04

        velocity_command_filt = np.zeros(3)
        angular_command_filt = 0.0
        alpha = 0.15

        init_ori = copy.deepcopy(np.array([self.cur_pose.pose.orientation.x,
                                           self.cur_pose.pose.orientation.y,
                                           self.cur_pose.pose.orientation.z,
                                           self.cur_pose.pose.orientation.w]))
        init_pos = copy.deepcopy(np.array([self.cur_pose.pose.position.x,
                                           self.cur_pose.pose.position.y,
                                           self.cur_pose.pose.position.z]))
        init_hand_ori = copy.deepcopy(np.array([self.goal_pose.pose.orientation.x,
                                                self.goal_pose.pose.orientation.y,
                                                self.goal_pose.pose.orientation.z,
                                                self.goal_pose.pose.orientation.w]))

        # [LEGACY ROS 1 not implemented]
        # r = np.linalg.norm(np.array([self.offset.pose.position.x,self.offset.pose.position.y]))

        start_time = time.time()
        while time.time() - start_time < 30.0:
            if goal_handle.is_cancel_requested:
                self.node.get_logger().info('Action preempted')
                return False

            current_position = np.array([self.cur_pose.pose.position.x,
                                         self.cur_pose.pose.position.y,
                                         self.cur_pose.pose.position.z])
            current_orientation = np.array([self.cur_pose.pose.orientation.x,
                                            self.cur_pose.pose.orientation.y,
                                            self.cur_pose.pose.orientation.z,
                                            self.cur_pose.pose.orientation.w])
            current_hand_ori = np.array([self.goal_pose.pose.orientation.x,
                                         self.goal_pose.pose.orientation.y,
                                         self.goal_pose.pose.orientation.z,
                                         self.goal_pose.pose.orientation.w])

            offset_w = np.array([self.offset.pose.position.x, self.offset.pose.position.y, self.offset.pose.position.z, 1.0])

            ## NEW - HAND ORIENTATION
            err_hand_ori = tf_trans.euler_from_quaternion(
                tf_trans.quaternion_multiply(current_hand_ori, tf_trans.quaternion_inverse(init_hand_ori)))

            #print(err_hand_ori)
            if err_hand_ori[2] > np.pi / 2:
                # err_hand_ori = (err_hand_ori[0], err_hand_ori[1], np.pi / 2)
                err_hand_ori[2] = np.pi/2
            if err_hand_ori[2] < -np.pi / 2:
                # err_hand_ori = (err_hand_ori[0], err_hand_ori[1], -np.pi / 2)
                err_hand_ori[2] = -np.pi/2

            Rz = tf_trans.euler_matrix(0, 0, err_hand_ori[2])
            offset_w = Rz.dot(offset_w)

            size_x = self.size[0] if self.size is not None else 0.0
            goal_position = np.array([self.goal_pose.pose.position.x + size_x / 2,
                                      self.goal_pose.pose.position.y,
                                      self.goal_pose.pose.position.z]) + offset_w[:3]
            ## END NEW
            '''
            goal_position = np.array([self.goal_pose.pose.position.x + self.size[0]/2,
                                        self.goal_pose.pose.position.y,
                                        self.goal_pose.pose.position.z]) + offset_w[:3]
            '''
            delta_x = goal_position[0] # - init_pos[0] 
            delta_y = goal_position[1] - init_pos[1]

            goal_yaw = np.arctan2(delta_y, delta_x)
            # goal_yaw = np.clip(goal_yaw, -0.25, 0.25)
            if goal_yaw > 0.25:
                goal_yaw = 0.25
            if goal_yaw < -0.25:
                goal_yaw = -0.25

            goal_quaternion = tf_trans.quaternion_from_euler(0, 0, goal_yaw) # -np.pi
            target_quaternion = tf_trans.quaternion_multiply(init_ori, goal_quaternion)
            current_orientation[3] = -current_orientation[3]
            error_quaternion = tf_trans.quaternion_multiply(target_quaternion, current_orientation)

            # if error_quaternion[3] < 0:
            #    error_quaternion = -error_quaternion
            ori_error = np.array(tf_trans.euler_from_quaternion(error_quaternion))
            position_error = goal_position - current_position
            position_integral += position_error * dt
            position_error_derivative = (position_error - self.last_err) / dt
            self.last_err = position_error

            # Calculate velocity command with PID
            raw_velocity_command = (self.Kp * position_error +
                                    self.Kd * position_error_derivative +
                                    self.Ki * position_integral)

            # Limit the velocity to prevent high initial jerk
            velocity_command = np.clip(raw_velocity_command, -max_velocity, max_velocity)
            max_velocity = min(max_velocity + velocity_ramp_rate, 1.0)
            # Gradually increase the max velocity limit
            velocity_command_filt += alpha * (velocity_command - velocity_command_filt)

            twist = TwistStamped()
            twist.header.stamp = self.node.get_clock().now().to_msg()
            twist.header.frame_id = self.frame_base
            twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = float(velocity_command_filt[0]), float(velocity_command_filt[1]), float(velocity_command_filt[2])

            orientation_integral += ori_error * dt
            ori_error_derivative = (ori_error - self.last_ori_err) / dt
            self.last_ori_err = ori_error

            raw_angular_velocity = (self.Kp_ori * ori_error +
                                     self.Kd_ori * ori_error_derivative +
                                     self.Ki_ori * orientation_integral)

            angular_velocity = np.clip(raw_angular_velocity, -max_angular_velocity, max_angular_velocity)
            max_angular_velocity = min(max_angular_velocity + angular_velocity_ramp_rate, 0.8)

            angular_command_filt += 0.1 * (angular_velocity[0] - angular_command_filt)

            twist.twist.angular.x = 0
            twist.twist.angular.y = 0
            twist.twist.angular.z = float(angular_command_filt)

            self.vel_pub.publish(twist)

            if np.linalg.norm(position_error) < 0.05: # 0.05: # Decrease this value for better precision
                self.node.get_logger().info('Goal reached!!')
                max_velocity = 0.15, velocity_command_filt
                return True

            time.sleep(dt)

        return False