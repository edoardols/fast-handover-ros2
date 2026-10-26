#! /usr/bin/env python3

import actionlib
import math
import rosgraph
import time
import rospy
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from sensor_msgs.msg import JointState, PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Twist, Quaternion
from vision_msgs.msg import BoundingBox3D
import tf.transformations as tf_trans
from tf.transformations import quaternion_slerp, quaternion_matrix, quaternion_from_matrix, euler_from_quaternion, quaternion_multiply, quaternion_inverse
import numpy as np
#from tqdm import tqdm
from std_msgs.msg import String, Float64, Float64MultiArray, Int8        
import tf2_ros
import tf2_geometry_msgs
from tf import transformations as tt
from handover_action.msg import JointAction, JointGoal, JointResult, JointFeedback,\
        PoseAction, PoseGoal, PoseResult, PoseFeedback,\
        GripperAction, GripperGoal, GripperResult, GripperFeedback

from std_srvs.srv import Trigger
from std_msgs.msg import String
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest
from copy import deepcopy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest  
import sys

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from franka_gripper.msg import MoveAction, MoveGoal, MoveActionGoal
from franka_gripper.msg import GraspAction, GraspGoal, GraspActionGoal
import franka_gripper 

from time import sleep

#from IPython import embed

class JointActionServer:

    def __init__(self, joint_config, controller1, movegroup):

        self.joint_config = joint_config
        self.controller1 = controller1
        self.movegroup = movegroup

        self.last_ee_pose = PoseStamped()

        self.max_seconds = 60
        self.active = False
        
        self.pub_traj_disp = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=2)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(self.movegroup)
        self.group.set_planner_id("RRTstarkConfigDefault")
        self.group.set_max_velocity_scaling_factor(1.0)
        #self.group.allow_replanning(True)
        #self.group.set_num_planning_attempts(50)
        #self.group.set_planning_time(10)

        self.a_server = actionlib.SimpleActionServer(
            "joint_action_server",
            JointAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.a_server.start()
        rospy.loginfo("Started Joint ActionServer")

        self.obj = CollisionObject()
        self.sub_planning = rospy.Subscriber("/move_group/monitored_planning_scene", PlanningScene, callback=self.planning_scene_callback, queue_size=1)
        self.apply_planning_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        self.apply_planning_scene_req = self.apply_planning_scene(PlanningScene(name=self.obj.id, is_diff=True))

        # Controller Switch Service
        rospy.wait_for_service('/controller_manager/switch_controller')
        self.switch_controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

    def switch_controllers(self,controller1,controller2):
        try:
            req = SwitchControllerRequest()
            req.start_controllers = [controller1]
            req.stop_controllers = [controller2]
            req.strictness = 0  # Best effort
            req.start_asap = True
            req.timeout = 2.0
            resp = self.switch_controller_service(req)
            return resp.ok
        except rospy.ServiceException as e:
            rospy.logerr("Controller switch service call failed: %s", e)
            return False
   
    def GotoJoint(self, joint):
        self.group.clear_pose_targets()
        self.group.set_joint_value_target(joint)

        # Get the plan and check the output
        planning_output = self.group.plan()
        planning_success = planning_output[0]  # Typically, the first element is the success flag
        if isinstance(planning_output, tuple) and len(planning_output) > 1:
            plan = planning_output[1]  # Assuming the second element is the plan if present
        else:
            plan = None

        if planning_success and plan and plan.joint_trajectory.points:
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)        
            self.pub_traj_disp.publish(display_trajectory)
            # Uncomment the following line if manual confirmation is needed
            #t = input('Execute the plan? (y/n): ')
            # if t.lower() == 'y':
            self.group.execute(plan, wait=True)
            return True
        else:
            rospy.logerr("Planning failed or no valid plan was generated.")
            return False


    def execute_cb(self, goal):

        if goal.rest.data:

            if not self.switch_controllers(self.controller1,'joint_group_vel_controller'):
                rospy.logerr("Failed to switch controllers")
                self.a_server.set_aborted()
                return

            rospy.loginfo("Going to home position")
            joint_config = self.joint_config 
            result = JointResult()
            res = self.GotoJoint(joint_config)
            result.res.data = res
            self.a_server.set_succeeded(result)

        else:

            if not self.switch_controllers(self.controller1,'joint_group_vel_controller'):
                rospy.logerr("Failed to switch controllers")
                self.a_server.set_aborted()
                return

            rospy.loginfo("Going to home position")
            joint_config = self.joint_config 
            result = JointResult()

            res = self.GotoJoint(joint_config)
            result.res.data = res
            
            if not self.switch_controllers('joint_group_vel_controller', self.controller1):
                rospy.logerr("Failed to switch controllers")
                self.a_server.set_aborted()
                return

            t=input('Go to next state? (press y)')
            if t.lower() == 'y':
                self.a_server.set_succeeded(result)
                print('Go to next state!')
        

    def planning_scene_callback(self, data):
        self.obj = data.world.collision_objects


class PoseActionServer:
    def __init__(self, frame_base, ee_frame):


        self.frame_base = frame_base
        self.ee_frame = ee_frame

        rospy.wait_for_service('/compute_fk',timeout=500)
        self.fk_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        
        self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=1)
        self.vel_publisher = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)
        self.pose_publisher = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)

        self.goal_pose_subscriber = rospy.Subscriber('/grasp_pose', PoseStamped, self.goal_pose_callback, queue_size=1) #/goal_pose
        self.vel_subscriber = rospy.Subscriber('/servo_server/delta_twist_cmds', TwistStamped, self.velocity_callback, queue_size=1)
        
        

        self.current_joint_state = None
        self.cur_pose = PoseStamped()
        self.goal_pose = PoseStamped()

        self.Kp = np.array([1.9, 1.9, 2.3]) # 1.8 1.8 2.2
        self.Ki = np.array([0.0, 0.0, 0.0])
        self.Kd = np.array([0.05, 0.05, 0.05])

        self.Kp_ori = np.array([0.2, 0.2, 0.2])
        self.Ki_ori = np.array([0.1, 0.1, 0.1])
        self.Kd_ori = np.array([0.001, 0.001, 0.001])
        
        self.last_err = np.zeros(3)
        self.last_ori_err = np.zeros(3)

        self.a_server = actionlib.SimpleActionServer(
            "pose_action_server",
            PoseAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.a_server.start()
        rospy.loginfo("Started Pose ActionServer")

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.update_current_pose()

    def goal_pose_callback(self, msg):
        self.goal_pose = msg

    def velocity_callback(self, msg):
        self.vel_cmd = msg

    def update_current_pose(self):
        if self.current_joint_state is None:
            return

        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = self.frame_base #'panda_link0'
        fk_request.fk_link_names = [self.ee_frame] #['robotiq_virtual_link']
        fk_request.robot_state.joint_state = self.current_joint_state

        try:
            response = self.fk_service(fk_request)
            if response.error_code.val == response.error_code.SUCCESS:
                self.cur_pose = response.pose_stamped[0]
                self.cur_pose.header.stamp = rospy.Time.now()
                self.pose_publisher.publish(self.cur_pose)
            else:
                rospy.logerr("Failed to perform FK: %s", response.error_code)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


    def WaitForGoal(self):
        try:
            goal = rospy.wait_for_message("/grasp_pose", PoseStamped, timeout=60)  # Wait for 60 seconds for a point cloud
        except rospy.ROSException as e:
            rospy.logerr("Failed to receive goal pose: %s", e)
            return None
        return goal


    def execute_cb(self, goal):    

        R_EE = tt.quaternion_matrix([self.cur_pose.pose.orientation.x, self.cur_pose.pose.orientation.y,\
                                     self.cur_pose.pose.orientation.z, self.cur_pose.pose.orientation.w])


        if goal.goal_pose.pose.position.x !=0 and goal.goal_pose.pose.position.y !=0 and goal.goal_pose.pose.position.z !=0:  # Verifying if a goal is sent directly with the action call
            rospy.loginfo('Received goal directly from action call.')
            self.goal_pose = goal.goal_pose
            
            if goal.flag.data:
                rospy.sleep(0.5)
                offset_world = R_EE.dot([0,0, goal.goal_pose.pose.position.z, 0]) 
                self.goal_pose = deepcopy(self.cur_pose)
                print(offset_world)
                self.goal_pose.pose.position.x += offset_world[0]
                self.goal_pose.pose.position.y += offset_world[1]
                self.goal_pose.pose.position.z += offset_world[2]                
            
        else:
            rospy.loginfo('Waiting for goal from /goal_pose topic.')
            goal = self.WaitForGoal()

        
            if self.goal_pose is None:
                self.a_server.set_aborted()
                rospy.logerr("No goal received, action aborted.")
                return

        success = self.control_loop() #goal.target_pose

        if success:
            self.a_server.set_succeeded(PoseResult(pose=self.cur_pose))
        else:
            self.a_server.set_aborted()

    def control_loop(self):
        dt = 1/50.0  # OLD: 50
        position_integral = 0
        orientation_integral = 0
        max_velocity = 0.06  # Start with a lower max velocity
        velocity_ramp_rate = 0.025  # Incremental increase per cycle
        rate = rospy.Rate(50)  # OLD: 50

        velocity_command_filt = np.array([self.vel_cmd.twist.linear.x,self.vel_cmd.twist.linear.y,self.vel_cmd.twist.linear.z]) #np.zeros(3)
        alpha = 0.5

        while not rospy.is_shutdown():
            if self.a_server.is_preempt_requested():
                rospy.loginfo('Action preempted')
                return False

            current_position = np.array([self.cur_pose.pose.position.x, 
                                         self.cur_pose.pose.position.y, 
                                         self.cur_pose.pose.position.z])
            goal_position = np.array([self.goal_pose.pose.position.x, 
                                      self.goal_pose.pose.position.y, 
                                      self.goal_pose.pose.position.z])

            position_error = goal_position - current_position
            position_integral += position_error * dt  # Update integral error
            position_error_derivative = (position_error - self.last_err) / dt
            self.last_err = position_error

            # Calculate velocity command with PID
            raw_velocity_command = self.Kp * position_error + \
                                   self.Kd * position_error_derivative + \
                                   self.Ki * position_integral

            # Limit the velocity to prevent high initial jerk
            velocity_command = np.clip(raw_velocity_command, -max_velocity, max_velocity)
            max_velocity = min(max_velocity + velocity_ramp_rate, 1.0)  # Gradually increase the max velocity limit

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

            velocity_command_filt += alpha*(velocity_command-velocity_command_filt)


            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = velocity_command_filt
            twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = angular_velocity_command

            twist.twist.angular.x = 0
            twist.twist.angular.y = 0
            twist.twist.angular.z = 0


            self.vel_publisher.publish(twist)

            if np.linalg.norm(position_error) < 0.03: # Check if the goal is reached
                rospy.loginfo('Goal reached')
                return True

            rate.sleep()


    
class CameraActionServer:

    def __init__(self, arm, frame_base, ee_frame):


        self.frame_base = frame_base
        self.ee_frame = ee_frame
        self.arm = arm

        rospy.wait_for_service('/compute_fk')
        self.fk_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)


        self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback, queue_size=1)
        self.vel_publisher = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)
        self.pose_publisher = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        self.goal_pose_subscriber = rospy.Subscriber('/hand_pose', PoseStamped, self.goal_pose_callback, queue_size=1) #hand_pose
        self.object_pose_subscriber = rospy.Subscriber('/object_pose', PoseStamped, self.object_pose_callback, queue_size=1) #hand_pose
        self.obj_bb_subscriber = rospy.Subscriber('/object_bb', BoundingBox3D, self.obj_bb_callback, queue_size=1)

       

        self.current_joint_state = None
        self.cur_pose = PoseStamped()
        self.goal_pose = PoseStamped()

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

        self.init_obj_bb = None
        self.offset = None
        self.size = None
        self.init_hand_pose = None
        self.optimal_z_offset = None
        

        self.rotated = False

 
        self.a_server = actionlib.SimpleActionServer(
            "camera_action_server",
            PoseAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.a_server.start()
        rospy.loginfo("Started Camera ActionServer")


    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.update_current_pose()

    def goal_pose_callback(self, msg):
        self.goal_pose = msg

    def object_pose_callback(self, msg):
        self.object_pose = msg


        

    def obj_bb_callback(self, msg):
        self.obj_bb = msg
        

    def update_current_pose(self):
        if self.current_joint_state is None:
            return

        fk_request = GetPositionFKRequest()
        fk_request.header.frame_id = self.frame_base 
        fk_request.fk_link_names = [self.ee_frame] 
        fk_request.robot_state.joint_state = self.current_joint_state

        try:
            response = self.fk_service(fk_request)
            if response.error_code.val == response.error_code.SUCCESS:
                self.cur_pose = response.pose_stamped[0]
                self.cur_pose.header.stamp = rospy.Time.now()
                self.pose_publisher.publish(self.cur_pose)
            else:
                rospy.logerr("Failed to perform FK: %s", response.error_code)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def ptCloud(self):
        try:
            obj = rospy.wait_for_message("/object_pointcloud", PointCloud2, timeout=500)  # Wait for 10 seconds for a point cloud
        except rospy.ROSException as e:
            rospy.logerr("Failed to receive point cloud: %s", e)
            return None
        return obj

    def objPose(self):
        try:
            obj_pose = rospy.wait_for_message("/object_pose", PoseStamped, timeout=50)  # Wait for 5 seconds for an object pose
            offset = PoseStamped()
            offset.pose.position.x = self.goal_pose.pose.position.x - obj_pose.pose.position.x
            offset.pose.position.y = self.goal_pose.pose.position.y - obj_pose.pose.position.y
            offset.pose.position.z = self.goal_pose.pose.position.z - obj_pose.pose.position.z

            return obj_pose, offset # Directly return the received pose
        except rospy.ROSException as e:
            rospy.logerr("Failed to receive object pose: %s", e)
            return None
        return obj_pose, offset


    def objBB(self):
        try:
            obj_bb = rospy.wait_for_message("/object_bb", BoundingBox3D, timeout=50)  # Wait for 5 seconds for an object pose
            offset = PoseStamped()
            offset.pose.position.x = obj_bb.center.position.x - self.goal_pose.pose.position.x
            offset.pose.position.y = obj_bb.center.position.y - self.goal_pose.pose.position.y 
            offset.pose.position.z = obj_bb.center.position.z - self.goal_pose.pose.position.z

            size = np.array([obj_bb.size.x,obj_bb.size.y,obj_bb.size.z])

            return obj_bb, offset, size # Directly return the received pose
        except rospy.ROSException as e:
            rospy.logerr("Failed to receive object pose: %s", e)
            return None
        return obj_bb, offset, size


    def execute_cb(self, goal):

        if self.init_obj_bb is None:
            self.init_obj_bb, _, self.size = self.objBB()  # Get the initial object pose
            rospy.logerr("No initial object bb received")
        
        #If the object is short and large the gripper is rotated 
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
        while (np.abs(self.goal_pose.pose.position.x-self.init_obj_bb.center.position.x) > self.size[0]/2 + 0.05  or \
              np.abs(self.goal_pose.pose.position.y-self.init_obj_bb.center.position.y) > self.size[1]/2 + 0.05  or \
              np.abs(self.goal_pose.pose.position.z-0.035-self.init_obj_bb.center.position.z) > self.size[2]/2 + 0.05 or \
              np.linalg.norm(np.array([self.goal_pose.pose.position.x,
                                       self.goal_pose.pose.position.y,
                                       self.goal_pose.pose.position.z-0.035]) -
                             np.array([self.init_obj_bb.center.position.x,
                                       self.init_obj_bb.center.position.y,
                                       self.init_obj_bb.center.position.z]))) > 0.035+ np.linalg.norm(self.size)/2  and \
              self.init_hand_pose is None:

              tmp = None

        if self.init_hand_pose is None:
            self.offset = PoseStamped()
            self.init_hand_pose = deepcopy(self.goal_pose)
            self.offset.pose.position.x = self.init_obj_bb.center.position.x - self.init_hand_pose.pose.position.x #+ self.size[0]/2
            self.offset.pose.position.y = self.init_obj_bb.center.position.y - self.init_hand_pose.pose.position.y 
            self.offset.pose.position.z = self.init_obj_bb.center.position.z - self.init_hand_pose.pose.position.z

            self.optimal_z_offset = self.calculate_z_offset(self.size[2],self.obj_bb.center.position.z,self.init_hand_pose.pose.position.z-0.035)
            if self.rotated == False:
                self.offset.pose.position.z += self.optimal_z_offset

        success = False
        
        init_hand_z = self.init_hand_pose.pose.position.z


        # Start hand tracking
        if  self.goal_pose.pose.position.z> init_hand_z +0.02: 
            success = self.control_loop() #goal.target_pose
            print('START HANDOVER')

        init_obj_pose = PoseStamped()
        init_obj_pose.pose.position.x = self.init_obj_bb.center.position.x
        init_obj_pose.pose.position.y = self.init_obj_bb.center.position.y
        init_obj_pose.pose.position.z = self.init_obj_bb.center.position.z
        if self.rotated:
            init_obj_pose.pose.position.z += 0.04
        else:
            init_obj_pose.pose.position.z += 0.7*self.optimal_z_offset

        if success:
            self.a_server.set_succeeded(PoseResult(pose=init_obj_pose))
            self.init_obj_bb = None
            self.init_hand_pose = None
            self.size = None
            self.rotated = False
            self.offset = None
        else:
            self.a_server.set_aborted()


    def calculate_z_offset(self, object_height, obj_z, hand_z):
        
        if hand_z > obj_z: 
            offset_z = -.35*object_height #0.46
        else:
            offset_z =  .4*object_height

        return offset_z


    
    def rotate_gripper(self, rotation, axis):
        dt = 1/50.0
        rate = rospy.Rate(50) 

        max_angular_velocity = 0.1  # Start with a lower max angular velocity
        angular_velocity_ramp_rate = 0.07  # Incremental increase per cycle

        angular_command_filt = np.zeros(3) 
        alpha = 0.5

        orientation_integral = 0
        Kp = np.array([2, 2, 2])
        Ki = np.array([0, 0, 0])
        Kd = np.array([0.05, 0.05, 0.05])
        
        current_orientation = np.array([self.cur_pose.pose.orientation.x,
                                    self.cur_pose.pose.orientation.y,
                                    self.cur_pose.pose.orientation.z,
                                    self.cur_pose.pose.orientation.w])

        target_quaternion = tf_trans.quaternion_multiply(current_orientation,rotation) #current_orientation*additional_rotation 


        while not rospy.is_shutdown():
            if self.a_server.is_preempt_requested():
                rospy.loginfo('Action preempted')
                return False

           
            current_orientation = np.array([self.cur_pose.pose.orientation.x,
                                        self.cur_pose.pose.orientation.y,
                                        self.cur_pose.pose.orientation.z,
                                        self.cur_pose.pose.orientation.w])


            ori_error = np.array(tf_trans.euler_from_quaternion(
                tf_trans.quaternion_multiply(target_quaternion, 
                                             tf_trans.quaternion_inverse(current_orientation))))
        
            orientation_integral += ori_error * dt
            ori_error_derivative = (ori_error - self.last_ori_err) / dt
            self.last_ori_err = ori_error


            raw_angular_velocity = Kp * ori_error + \
                                     Kd * ori_error_derivative + \
                                     Ki * orientation_integral 
                      
            #print('raw',raw_angular_velocity_x)               
            angular_velocity = np.clip(raw_angular_velocity, -max_angular_velocity, max_angular_velocity)
            max_angular_velocity = min(max_angular_velocity + angular_velocity_ramp_rate, 1.5)

            angular_command_filt += alpha*(angular_velocity-angular_command_filt) #(raw_angular_velocity_x-angular_command_filt) #

            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x=0
            twist.twist.linear.y=0
            twist.twist.linear.z=0

            if axis == 'x':
                twist.twist.angular.x = angular_command_filt[0]            
                twist.twist.angular.y = 0 #angular_command_filt[1]
                twist.twist.angular.z = 0 #angular_command_filt[2]

                if np.linalg.norm(ori_error[0])<0.01 : 
                    rospy.loginfo('EE rotated!')
                    return True
            elif axis == 'y':
                twist.twist.angular.x = 0            
                twist.twist.angular.y = angular_command_filt[1]
                twist.twist.angular.z = 0

                if np.linalg.norm(ori_error[1])<0.01 : 
                    rospy.loginfo('EE rotated!')
                    return True

            elif axis == 'z':
                twist.twist.angular.x = 0            
                twist.twist.angular.y = 0
                twist.twist.angular.z = angular_command_filt[2]

            self.vel_publisher.publish(twist)
            
            '''
            if np.linalg.norm(ori_error[0])<0.01 : 
                rospy.loginfo('EE rotated!')
                return True
            '''
            rate.sleep()


    def control_loop(self):
        dt = 1/50.0   # OLD: 50
        rate = rospy.Rate(50)    # OLD: 50

        position_integral = 0
        orientation_integral = 0
        max_velocity = 0.15  
        velocity_ramp_rate = 0.025  
        
        max_angular_velocity = 0.03  
        angular_velocity_ramp_rate = 0.04  

        velocity_command_filt = np.zeros(3)
        angular_command_filt = 0
        alpha = 0.15 


        init_ori = deepcopy(np.array([self.cur_pose.pose.orientation.x,
                                            self.cur_pose.pose.orientation.y,
                                            self.cur_pose.pose.orientation.z,
                                            self.cur_pose.pose.orientation.w]))

        init_pos = deepcopy(np.array([self.cur_pose.pose.position.x,
                                            self.cur_pose.pose.position.y,
                                            self.cur_pose.pose.position.z]))

        init_hand_ori = deepcopy(np.array([self.goal_pose.pose.orientation.x,
                                            self.goal_pose.pose.orientation.y,
                                            self.goal_pose.pose.orientation.z,
                                            self.goal_pose.pose.orientation.w]))


        r = np.linalg.norm(np.array([self.offset.pose.position.x,self.offset.pose.position.y]))
            

        while not rospy.is_shutdown():
            if self.a_server.is_preempt_requested():
                rospy.loginfo('Action preempted')
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
           
            #offset_w = np.array([self.offset.pose.position.x, self.offset.pose.position.y, self.offset.pose.position.z])
            offset_w = np.array([self.offset.pose.position.x, self.offset.pose.position.y, self.offset.pose.position.z, 1])

            ## NEW - HAND ORIENTATION
            
            err_hand_ori = tf_trans.euler_from_quaternion(
                tf_trans.quaternion_multiply(current_hand_ori, 
                                             tf_trans.quaternion_inverse(init_hand_ori)))

            #print(err_hand_ori)
            if err_hand_ori[2] > np.pi/2:
                err_hand_ori[2] = np.pi/2
            if err_hand_ori[2] < -np.pi/2:
                err_hand_ori[2] = -np.pi/2
        
            Rz = tt.euler_matrix(0,0,err_hand_ori[2])
            
            offset_w = Rz.dot(offset_w)

            goal_position = np.array([self.goal_pose.pose.position.x + self.size[0]/2,
                                      self.goal_pose.pose.position.y,
                                      self.goal_pose.pose.position.z]) + offset_w[:3]

            ## END NEW

            '''
            goal_position = np.array([self.goal_pose.pose.position.x + self.size[0]/2,
                                      self.goal_pose.pose.position.y,
                                      self.goal_pose.pose.position.z]) + offset_w[:3]
            '''
            
            delta_x = goal_position[0] #- init_pos[0] 
            delta_y = goal_position[1] - init_pos[1] 

           
            goal_yaw = np.arctan2(delta_y, delta_x)
            if goal_yaw > 0.25:
                goal_yaw = 0.25
            if goal_yaw < -0.25:
                goal_yaw = -0.25
        
            goal_quaternion = tf_trans.quaternion_from_euler(0, 0, goal_yaw) #-np.pi

            target_quaternion = tf_trans.quaternion_multiply(init_ori,goal_quaternion) 
            current_orientation[3] = -current_orientation[3]
            error_quaternion = tf_trans.quaternion_multiply(target_quaternion,current_orientation)
            
            #if error_quaternion[3] < 0:
            #    error_quaternion = -error_quaternion
            ori_error = np.array(tf_trans.euler_from_quaternion(error_quaternion))
            
            position_error = goal_position - current_position
            position_integral += position_error * dt  
            position_error_derivative = (position_error - self.last_err) / dt
            self.last_err = position_error

            # Calculate velocity command with PID
            raw_velocity_command = self.Kp * position_error + \
                                   self.Kd * position_error_derivative + \
                                   self.Ki * position_integral

            # Limit the velocity to prevent high initial jerk
            velocity_command = np.clip(raw_velocity_command, -max_velocity, max_velocity)
            max_velocity = min(max_velocity + velocity_ramp_rate, 1.0)  # Gradually increase the max velocity limit

            velocity_command_filt += alpha*(velocity_command-velocity_command_filt)

            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = velocity_command_filt
            
            orientation_integral += ori_error * dt
            ori_error_derivative = (ori_error - self.last_ori_err) / dt
            self.last_ori_err = ori_error

            raw_angular_velocity = self.Kp_ori * ori_error + \
                                     self.Kd_ori * ori_error_derivative + \
                                     self.Ki_ori * orientation_integral


            angular_velocity = np.clip(raw_angular_velocity, -max_angular_velocity, max_angular_velocity)
            max_angular_velocity = min(max_angular_velocity + angular_velocity_ramp_rate, 0.8)

            angular_command_filt += 0.1*(angular_velocity-angular_command_filt)

            twist.twist.angular.z = angular_command_filt[0]
            
            twist.twist.angular.x = 0
            twist.twist.angular.y = 0

            self.vel_publisher.publish(twist)

            if np.linalg.norm(position_error) < 0.05: # 0.05: # Decrease this value for better precision
                rospy.loginfo('Goal reached!!')
                max_velocity = 0.15 
                return True, velocity_command_filt

            rate.sleep()    


class GripperActionServer:

    def __init__(self, arm):

        self.max_seconds = 60
        self.active = False
        self.arm = arm

        self.a_server = actionlib.SimpleActionServer(
            "gripper_action_server",
            GripperAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.a_server.start()
        rospy.loginfo("Started Gripper ActionServer")

        if self.arm == "ur5":
            self.pub_gripper = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
            self.robotiq_activation()

        elif self.arm == "panda":
            self.pub_gripper = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal)
            self.client0 = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)
            self.client1 = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

        #self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.cur_pose_subscriber = rospy.Subscriber('/current_pose', PoseStamped, self.cur_pose_callback )
      
        self.current_joint_state = None


    def robotiq_activation(self):

        grp_activ = outputMsg.Robotiq2FGripper_robot_output()
        grp_activ.rACT = 1
        grp_activ.rGTO = 1
        grp_activ.rSP  = 255
        grp_activ.rFR  = 0

        self.pub_gripper.publish(grp_activ)

    def panda_gripper_goal(self, width, speed):

        self.client1.wait_for_server()

        goal = franka_gripper.msg.GraspGoal()
        goal.width = width
        goal.epsilon.inner = 0.1
        goal.epsilon.outer = 0.1
        goal.speed = speed
        goal.force = 1.0

        self.client1.send_goal(goal)

        self.client1.wait_for_result()

        return self.client1.get_result() 

    def panda_gripper_open(self, width, speed):

        self.client0.wait_for_server()

        goal = franka_gripper.msg.MoveGoal()
        goal.width = width
        goal.speed = speed

        self.client0.send_goal(goal)

        self.client0.wait_for_result()

        return self.client0.get_result() 


    def execute_cb(self, goal):

        self.active = True
        

        result = GripperResult()

        if self.arm == "ur5":
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rATR = 0
            command.rSP  = 255 # closure speed
            command.rFR  = 0  # force
            command.rPR = int(goal.cmd_gripper.data)

            rate = rospy.Rate(10)
            elapsed = 0
            success = False
            while elapsed < self.max_seconds and not rospy.is_shutdown():
                # Check if preempted (if so, abort)
                if self.a_server.is_preempt_requested():
                    self.a_server.set_preempted()
                    self.active = False
                    return
                else:
                    
                    self.pub_gripper.publish(command)
                    rospy.sleep(0.5)
                    success = True
                    break
                
                elapsed += 1
                rate.sleep()

            self.active = False
            if success:
                result.current_pose = self.cur_pose
                self.a_server.set_succeeded(result)
            else:
                print("Aborted")
                self.a_server.set_aborted()

        elif self.arm == "panda":
            
            rate = rospy.Rate(10)
            elapsed = 0
            success = False

            while elapsed < self.max_seconds and not rospy.is_shutdown():
                # Check if preempted (if so, abort)
                if self.a_server.is_preempt_requested():
                    self.a_server.set_preempted()
                    self.active = False
                    return
                else:
                    
                    if goal.flag.data:
                        self.panda_gripper_open(float(goal.cmd_gripper.data),speed=0.8)
                    else:
                        self.panda_gripper_goal(float(goal.cmd_gripper.data),speed=0.8)


                    #rospy.sleep(0.5)
                    success = True
                    break
                
                elapsed += 1
                rate.sleep()

            self.active = False
            if success:
                result.current_pose = self.cur_pose
                self.a_server.set_succeeded(result)
            else:
                print("Aborted")
                self.a_server.set_aborted()


    def cur_pose_callback(self, msg):
        self.cur_pose = msg


if __name__ == "__main__":

    if not rosgraph.is_master_online():
        print('Error: ROS master not running')
        exit(1)

    rospy.init_node("handover_action_server")

    robot = rospy.get_param("~robot", "ur5")
    rospy.loginfo("Arm: %s", robot)

    if robot == "ur5":
        frame_base = "base_link"
        ee_frame = "robotiq_virtual_link"
        controller1 = "vel_joint_traj_controller"
        movegroup = "manipulator"
        joint_config = [-0.07, -2.23, 2.32, -0.087, 1.57, 3.14] #initial config ur5
    elif robot == "panda":
        frame_base = "panda_link0"
        ee_frame = "panda_hand_tcp"
        controller1 = "effort_joint_trajectory_controller"
        movegroup = "panda_manipulator"
        #joint_config = [0.0, -0.5, 0.06, -2.45, 0.0, 2.0, 0.0] #initial config panda
        joint_config = [0.0, -0.87, 0.028, -2.86, 0.0, 1.6, 0.0] #initial config panda



    else:
        rospy.logerr("Robotic arm not supported: %s", robot)
        sys.exit(1)


    joint_act_srv = JointActionServer(joint_config=joint_config, controller1=controller1, movegroup=movegroup)
    pose_act_srv = PoseActionServer(frame_base=frame_base, ee_frame=ee_frame)
    gripper_act_srv = GripperActionServer(arm=robot)
    camera_act_srv = CameraActionServer(arm=robot, frame_base=frame_base, ee_frame=ee_frame)

    rospy.spin()
