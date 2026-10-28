#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
import smach
import smach_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
import numpy as np

from handover_interfaces.action import Joint, Pose as PoseAct, Gripper


class StateBase(smach.State):
    def __init__(self, outcomes):
        super().__init__(outcomes=outcomes)

    def get_pose_goal(self, pose_data):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pose_data[:3]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = pose_data[3:]
        return pose


class Select(StateBase):
    def __init__(self, state_machine_context, node):
        super().__init__(outcomes=['selected', 'out_of_cycles'])
        self.state_machine_context = state_machine_context
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Selecting next action...')
        if self.state_machine_context['current_cycle'] < self.state_machine_context['max_cycles']:
            self.state_machine_context['current_cycle'] += 1
            return 'selected'
        else:
            return 'out_of_cycles'


class MoveToInitial(StateBase):
    def __init__(self, node):
        super().__init__(outcomes=['succeeded', 'failed'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info('Moving to initial pose...')
        return 'succeeded'


def main(args=None):
    rclpy.init(args=args)
    node = Node('handover_state_machine')

    node.declare_parameter("robot", "ur5")
    node.declare_parameter("position_based_grasp", True)

    robot = node.get_parameter("robot").get_parameter_value().string_value
    position_based_grasp = node.get_parameter("position_based_grasp").get_parameter_value().bool_value

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    sm = smach.StateMachine(outcomes=['finished', 'failed'])

    state_machine_context = {
        'current_cycle': 0,
        'max_cycles': 500
    }

    with sm:
        smach.StateMachine.add('SELECT', Select(state_machine_context, node),
                               transitions={'selected': 'HANDOVER', 'out_of_cycles': 'REST'})

        # Submachine for human-to-robot handover
        sm_handover = smach.StateMachine(outcomes=['placed', 'failed'], output_keys=['obj_pose'])

        # Open the state machine
        with sm_handover:

            def movetohome_cb(userdata, goal):
                movetojoint_goal = Joint.Goal()
                movetojoint_goal.rest = False
                return movetojoint_goal

            def movetohome_result_cb(userdata, status, result):
                if status == GoalStatus.STATUS_SUCCEEDED:
                    return 'succeeded'
                elif status == GoalStatus.STATUS_CANCELED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('MOVE_TO_HOME', smach_ros.SimpleActionState(
                                       node,
                                       'joint_action_server',
                                       Joint,
                                       goal_cb=movetohome_cb,
                                       result_cb=movetohome_result_cb,
                                       input_keys=['start']
                                   ),
                                   transitions={'succeeded': 'HAND_TRACKING',
                                                'preempted': 'failed',
                                                'aborted': 'failed'})

            def tracking_cb(userdata, goal):
                movetopose_goal = PoseAct.Goal()
                # t=raw_input('Go?')
                userdata.handover_time = time.time()
                node.get_logger().info(f"Start time: {userdata.handover_time:.3f} sec")
                return movetopose_goal

            def tracking_result_cb(userdata, status, result):
                if status == GoalStatus.STATUS_SUCCEEDED:
                    userdata.obj_pose = result.pose # target pose -> FINAL_POS
                    return 'succeeded'
                elif status == GoalStatus.STATUS_CANCELED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('HAND_TRACKING', smach_ros.SimpleActionState(
                                       node,
                                       'camera_action_server',
                                       PoseAct,
                                       goal_cb=tracking_cb,
                                       result_cb=tracking_result_cb,
                                       input_keys=['obj_pose', 'handover_time'],
                                       output_keys=['obj_pose', 'handover_time']
                                   ),
                                   transitions={'succeeded': 'GRASP',
                                                'preempted': 'failed',
                                                'aborted': 'HAND_TRACKING'},
                                   remapping={'obj_pose': 'obj_pose', 'handover_time': 'handover_time'})

            def grasp_goal_cb(userdata, goal):
                gripper_goal = Gripper.Goal()
                elapsed = time.time() - userdata.handover_time
                node.get_logger().info(f"Human maneuvering time: {elapsed:.3f} sec")
                userdata.handover_time = time.time()

                if robot == "ur5":
                    gripper_goal.cmd_gripper = 150.0 #180 #ROBOTIQ
                elif robot == "panda":
                    gripper_goal.cmd_gripper = 0.02 # 0.015 #PANDA GRIPPER
                    if position_based_grasp:
                        gripper_goal.flag = True # Use MoveAction instead of GraspAction

                return gripper_goal

            def grasp_result_cb(userdata, status, result):
                if status == GoalStatus.STATUS_SUCCEEDED:
                    return 'succeeded'
                elif status == GoalStatus.STATUS_CANCELED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('GRASP', smach_ros.SimpleActionState(
                                       node,
                                       'gripper_action_server',
                                       Gripper,
                                       goal_cb=grasp_goal_cb,
                                       result_cb=grasp_result_cb,
                                       input_keys=['grasp', 'handover_time'],
                                       output_keys=['handover_time']
                                   ),
                                   transitions={'succeeded': 'FINAL_POSE',
                                                'preempted': 'failed',
                                                'aborted': 'failed'},
                                   remapping={'obj_pose': 'obj_pose', 'handover_time': 'handover_time'})

            def finalpose_goal_cb(userdata, goal):
                place_goal = PoseAct.Goal()
                place_goal.goal_pose = userdata.obj_pose
                place_goal.goal_pose.pose.position.z = max(place_goal.goal_pose.pose.position.z, 0.015)
                place_goal.flag = False
                elapsed = time.time() - userdata.handover_time
                node.get_logger().info(f"Handover time: {elapsed:.3f} sec")

                userdata.handover_time = time.time()
                return place_goal

            def finalpose_result_cb(userdata, status, result):
                if status == GoalStatus.STATUS_SUCCEEDED:
                    return 'succeeded'
                elif status == GoalStatus.STATUS_CANCELED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('FINAL_POSE', smach_ros.SimpleActionState(
                                       node,
                                       'pose_action_server',
                                       PoseAct,
                                       goal_cb=finalpose_goal_cb,
                                       result_cb=finalpose_result_cb,
                                       input_keys=['obj_pose', 'handover_time'],
                                       output_keys=['handover_time']
                                   ),
                                   transitions={'succeeded': 'PLACE',
                                                'preempted': 'failed',
                                                'aborted': 'failed'},
                                   remapping={'handover_time': 'handover_time'})

            def place_goal_cb(userdata, goal):
                gripper_goal = Gripper.Goal()
                if robot == "ur5":
                    gripper_goal.cmd_gripper = 0.0 # ROBOTIQ
                elif robot == "panda":
                    gripper_goal.cmd_gripper = 0.08 # PANDA GRIPPER
                    gripper_goal.flag = True # Use MoveAction instead of GraspAction

                return gripper_goal

            def place_result_cb(userdata, status, result):
                if status == GoalStatus.STATUS_SUCCEEDED:
                    userdata.target_pose = result
                    elapsed = time.time() - userdata.handover_time
                    node.get_logger().info(f"Robot maneuvering time: {elapsed:.3f} sec")
                    return 'succeeded'
                elif status == GoalStatus.STATUS_CANCELED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('PLACE', smach_ros.SimpleActionState(
                                       node,
                                       'gripper_action_server',
                                       Gripper,
                                       goal_cb=place_goal_cb,
                                       result_cb=place_result_cb,
                                       input_keys=['target_pose', 'handover_time'],
                                       output_keys=['target_pose', 'handover_time']
                                   ),
                                   transitions={'succeeded': 'GO_BACK',
                                                'preempted': 'failed',
                                                'aborted': 'failed'},
                                   remapping={'target_pose': 'target_pose', 'handover_time': 'handover_time'})

            def goback_goal_cb(userdata, goal):
                goback_goal = PoseAct.Goal()
                #EE displacement 
                goback_goal.goal_pose.pose.position.x = 0.01
                goback_goal.goal_pose.pose.position.y = 0.01
                goback_goal.goal_pose.pose.position.z = -0.09
                goback_goal.flag = True # It is used to transform the EE displacement in the world frame
                return goback_goal

            def goback_result_cb(userdata, status, result):
                if status == GoalStatus.STATUS_SUCCEEDED:
                    if robot == "panda":
                        time.sleep(2)
                    return 'succeeded'
                elif status == GoalStatus.STATUS_CANCELED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('GO_BACK', smach_ros.SimpleActionState(
                                       node,
                                       'pose_action_server',
                                       PoseAct,
                                       goal_cb=goback_goal_cb,
                                       result_cb=goback_result_cb,
                                       input_keys=['target_pose', 'handover_time'],
                                       output_keys=['handover_time']
                                   ),
                                   transitions={'succeeded': 'placed',
                                                'preempted': 'failed',
                                                'aborted': 'failed'})

        def rest_goal_cb(userdata, goal):
            rest_goal = Joint.Goal()
            rest_goal.rest = True
            return rest_goal

        smach.StateMachine.add('REST', smach_ros.SimpleActionState(
                                   node,
                                   'joint_action_server',
                                   Joint,
                                   goal_cb=rest_goal_cb
                               ),
                               transitions={'succeeded': 'finished',
                                            'preempted': 'failed',
                                            'aborted': 'failed'})

        smach.StateMachine.add('HANDOVER', sm_handover,
                               transitions={'placed': 'SELECT',
                                            'failed': 'failed'})

    # [LEGACY ROS 1 IntrospectionServer]
    # sis = smach_ros.IntrospectionServer('sm_introspection_server', sm, '/SM_ROOT')
    # sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    node.get_logger().info("Outcome: " + outcome)

    rclpy.shutdown()


if __name__ == '__main__':
    main()