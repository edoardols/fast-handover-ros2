#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.msg import PlanningScene, DisplayTrajectory, CollisionObject
from handover_interfaces.action import Joint

# [LEGACY ROS 1]
# import moveit_commander
# import moveit_msgs.msg
# from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest


class JointActionServer:
    def __init__(self, node, joint_config, controller1, movegroup):
        self.node = node
        self.joint_config = joint_config
        self.controller1 = controller1
        self.movegroup = movegroup
        self.cb_group = ReentrantCallbackGroup()

        self.last_ee_pose = PoseStamped()
        self.max_seconds = 60
        self.active = False

        self.pub_traj_disp = self.node.create_publisher(
            DisplayTrajectory,
            '/move_group/display_planned_path',
            2
        )

        # [LEGACY ROS 1]
        # moveit_commander.roscpp_initialize(sys.argv)
        # self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()
        # self.group = moveit_commander.MoveGroupCommander(self.movegroup)
        # self.group.set_planner_id("RRTstarkConfigDefault")
        # self.group.set_max_velocity_scaling_factor(1.0)
        # # self.group.allow_replanning(True)
        # # self.group.set_num_planning_attempts(50)
        # # self.group.set_planning_time(10)
        self.traj_pub = self.node.create_publisher(
            JointTrajectory,
            f'/{self.controller1}/joint_trajectory',
            10
        )

        self.obj = CollisionObject()
        # [LEGACY ROS 1]
        # self.sub_planning = rospy.Subscriber("/move_group/monitored_planning_scene", PlanningScene, callback=self.planning_scene_callback, queue_size=1)
        self.sub_planning = self.node.create_subscription(
            PlanningScene,
            '/move_group/monitored_planning_scene',
            self.planning_scene_callback,
            1,
            callback_group=self.cb_group
        )

        self.scene_pub = self.node.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        self.switch_controller_cli = self.node.create_client(
            SwitchController,
            '/controller_manager/switch_controller',
            callback_group=self.cb_group
        )

        self._action_server = ActionServer(
            self.node,
            Joint,
            'joint_action_server',
            execute_callback=self.execute_cb,
            callback_group=self.cb_group
        )

        self.node.get_logger().info("Started Joint ActionServer")

    def planning_scene_callback(self, msg: PlanningScene):
        self.obj = msg.world.collision_objects

    def apply_planning_scene(self):
        # [LEGACY ROS 1]
        # self.apply_planning_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        # self.apply_planning_scene_req = self.apply_planning_scene(PlanningScene(name=self.obj.id, is_diff=True))
        scene_msg = PlanningScene()
        scene_msg.name = self.obj.id if hasattr(self.obj, 'id') else ''
        scene_msg.is_diff = True
        self.scene_pub.publish(scene_msg)

    async def switch_controllers(self, start_controller, stop_controller):
        if not self.switch_controller_cli.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error("SwitchController service non available on /controller_manager")
            return False

        req = SwitchController.Request()
        req.activate_controllers = [start_controller] if start_controller else []
        req.deactivate_controllers = [stop_controller] if stop_controller else []
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        try:
            future = self.switch_controller_cli.call_async(req)
            while not future.done():
                await rclpy.utilities.sleep_for(0.1)
            res = future.result()
            return res.ok if res is not None else False
        except Exception as e:
            self.node.get_logger().error(f"Controller switch service call failed: {e}")
            return False

    def goto_joint(self, joint_values):
        # [LEGACY ROS 1 based on moveit_commander]
        # self.group.clear_pose_targets()
        # self.group.set_joint_value_target(joint)

        # # Get the plan and check the output
        # planning_output = self.group.plan()
        # planning_success = planning_output[0]  # Typically, the first element is the success flag
        # if isinstance(planning_output, tuple) and len(planning_output) > 1:
        #     plan = planning_output[1]  # Assuming the second element is the plan if present
        # else:
        #     plan = None

        # if planning_success and plan and plan.joint_trajectory.points:
        #     display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        #     display_trajectory.trajectory_start = self.robot.get_current_state()
        #     display_trajectory.trajectory.append(plan)        
        #     self.pub_traj_disp.publish(display_trajectory)
        #     # Uncomment the following line if manual confirmation is needed
        #     #t = input('Execute the plan? (y/n): ')
        #     # if t.lower() == 'y':
        #     self.group.execute(plan, wait=True)
        #     return True
        # else:
        #     rospy.logerr("Planning failed or no valid plan was generated.")
        #     return False
    
        msg = JointTrajectory()
        msg.header.stamp = self.node.get_clock().now().to_msg()

        if "panda" in self.movegroup.lower():
            msg.joint_names = [f"panda_joint{i+1}" for i in range(7)]
        else:
            msg.joint_names = [
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
            ]

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in joint_values]
        point.velocities = [0.0] * len(joint_values)
        point.accelerations = [0.0] * len(joint_values)
        point.time_from_start.sec = 4
        point.time_from_start.nanosec = 0
        
        msg.points.append(point)
        self.traj_pub.publish(msg)

    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        result = Joint.Result()

        if goal.rest:
            if not await self.switch_controllers(self.controller1, 'joint_group_vel_controller'):
                self.node.get_logger().error("Failed to switch controllers")
                goal_handle.abort()
                result.res = False
                return result

            self.node.get_logger().info("Going to home position")
            self.goto_joint(self.joint_config)
            # coherent with the time_from_start in the JointTrajectoryPoint row 161
            time.sleep(4.5)
            result.res = True
            goal_handle.succeed()
            return result
        else:
            if not await self.switch_controllers(self.controller1, 'joint_group_vel_controller'):
                self.node.get_logger().error("Failed to switch controllers")
                goal_handle.abort()
                result.res = False
                return result

            self.node.get_logger().info("Going to home position")
            self.goto_joint(self.joint_config)
            # coherent with the time_from_start in the JointTrajectoryPoint row 161
            time.sleep(4.5)

            if not await self.switch_controllers('joint_group_vel_controller', self.controller1):
                self.node.get_logger().error("Failed to switch controllers")
                goal_handle.abort()
                result.res = False
                return result
            
            # [LEGACY ROS 1]
            # t=input('Go to next state? (press y)')
            # if t.lower() == 'y':
            #     self.a_server.set_succeeded(result)
            #     print('Go to next state!')

            result.res = True
            goal_handle.succeed()
            return result