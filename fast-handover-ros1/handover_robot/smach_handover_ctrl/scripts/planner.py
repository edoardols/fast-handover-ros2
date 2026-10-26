#!/usr/bin/env python3

import rosgraph
import rospy
import smach
import smach_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_euler
import numpy as np
from handover_action.msg import \
     JointAction, JointGoal, JointResult, JointFeedback, \
     PoseAction, PoseGoal, PoseResult, PoseFeedback, \
     GripperAction, GripperGoal, GripperResult, GripperFeedback

class StateBase(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes)

    def get_pose_goal(self, pose_data):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pose_data[:3]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = pose_data[3:]
        return pose

class Select(StateBase):
    def __init__(self, state_machine_context):
        super().__init__(outcomes=['selected', 'out_of_cycles'])
        self.state_machine_context = state_machine_context

    def execute(self, userdata):
        rospy.loginfo('Selecting next action...')
        if self.state_machine_context['current_cycle'] < self.state_machine_context['max_cycles']:
            self.state_machine_context['current_cycle'] += 1
            return 'selected'
        else:
            return 'out_of_cycles'

class MoveToInitial(StateBase):
    def __init__(self):
        super().__init__(outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Moving to initial pose...')
        return 'succeeded'  


def main():

    rospy.init_node('handover_state_machine')
    robot = rospy.get_param("~robot", "ur5")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished', 'failed'])

    state_machine_context = {
    'current_cycle': 0,
    'max_cycles': 500
    }
    
    with sm:
        smach.StateMachine.add('SELECT', Select(state_machine_context), transitions={'selected': 'HANDOVER', 'out_of_cycles': 'REST'})
    

        # Submachine for human-to-robot handover
        sm_handover = smach.StateMachine(outcomes=['placed', 'failed'],
                                         output_keys=['obj_pose']) 
      
        # Open the state machine
        with sm_handover:

            
            def movetohome_cb(userdata, goal):
                movetojoint_goal = JointGoal() 
                movetojoint_goal.rest.data = False               
                return movetojoint_goal

            def movetohome_result_cb(userdata, status, result):
                if status == GoalStatus.SUCCEEDED:
                    return 'succeeded'
                elif status == GoalStatus.PREEMPTED:
                    return 'preempted'
                else:
                    return 'aborted'

            # Add states to the container
            smach.StateMachine.add('MOVE_TO_HOME',smach_ros.SimpleActionState(
                                       'joint_action_server',
                                       JointAction,
                                       goal_cb=movetohome_cb,
                                       result_cb=movetohome_result_cb,
                                       input_keys=['start']
                                   ),
                                   transitions={'succeeded': 'HAND_TRACKING',
                                                'preempted': 'failed',
                                                'aborted': 'failed'})

            def tracking_cb(userdata, goal):
                movetopose_goal = PoseGoal()
                #t=raw_input('Go?')
                userdata.handover_time = rospy.Time.now().to_sec()
                print('Start time',userdata.handover_time)
                return movetopose_goal

            def tracking_result_cb(userdata, status, result):
                if status == GoalStatus.SUCCEEDED:
                    userdata.obj_pose = result.pose #target pose -> FINAL_POSE
                    
                    return 'succeeded'
                elif status == GoalStatus.PREEMPTED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('HAND_TRACKING',smach_ros.SimpleActionState(
                                       'camera_action_server',
                                       PoseAction,
                                       goal_cb=tracking_cb,
                                       result_cb=tracking_result_cb,
                                       input_keys=['obj_pose','handover_time'],
                                       output_keys=['obj_pose','handover_time']
                                   ),
                                   transitions={'succeeded': 'GRASP',
                                                'preempted': 'failed',
                                                'aborted': 'HAND_TRACKING'},
                                    remapping={'obj_pose':'obj_pose',
                                                'handover_time':'handover_time'})

            def grasp_goal_cb(userdata, goal):
                gripper_goal = GripperGoal()
                elapsed = (rospy.Time.now().to_sec() - userdata.handover_time)
                rospy.loginfo("Human maneuvering time: %.3f sec", elapsed) 
                userdata.handover_time = rospy.Time.now().to_sec()

                if robot=="ur5":
                    gripper_goal.cmd_gripper.data = 150 #180 #ROBOTIQ
                elif robot=="panda":
                    gripper_goal.cmd_gripper.data = 0.02 # 0.015 #PANDA GRIPPER
                    position_based_grasp = rospy.get_param("position_based_grasp", True)
                    if position_based_grasp:
                        gripper_goal.flag.data = True # Use MoveAction instead of GraspAction

                return gripper_goal

            def grasp_result_cb(userdata, status, result):
                if status == GoalStatus.SUCCEEDED:
                    return 'succeeded'
                elif status == GoalStatus.PREEMPTED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('GRASP',
                                   smach_ros.SimpleActionState(
                                       'gripper_action_server',
                                       GripperAction,
                                       goal_cb=grasp_goal_cb,
                                       result_cb=grasp_result_cb,
                                       input_keys=['grasp','handover_time'],
                                       output_keys=['handover_time']

                                   ),
                                   transitions={'succeeded': 'FINAL_POSE',
                                                'preempted': 'failed',
                                                'aborted': 'failed'},
                                    remapping={'obj_pose':'obj_pose','handover_time':'handover_time'})

            def finalpose_goal_cb(userdata, goal):
                place_goal = PoseGoal()                
                place_goal.goal_pose = userdata.obj_pose
                place_goal.goal_pose.pose.position.z = max(place_goal.goal_pose.pose.position.z,0.015)
                place_goal.flag.data = False
                elapsed = (rospy.Time.now().to_sec() - userdata.handover_time)
                rospy.loginfo("Handover time: %.3f sec", elapsed) 

                userdata.handover_time = rospy.Time.now().to_sec()
                return place_goal

            def finalpose_result_cb(userdata, status, result):
                if status == GoalStatus.SUCCEEDED:
                    return 'succeeded'
                elif status == GoalStatus.PREEMPTED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('FINAL_POSE',
                                   smach_ros.SimpleActionState(
                                       'pose_action_server',
                                       PoseAction,
                                       goal_cb=finalpose_goal_cb,
                                       input_keys=['obj_pose','handover_time'],
                                       output_keys=['handover_time']
                                    ),
                                    transitions={'succeeded': 'PLACE',
                                                'preempted': 'failed',
                                                'aborted': 'failed'},
                                    remapping={'handover_time': 'handover_time'})

            def place_goal_cb(userdata, goal):
                gripper_goal = GripperGoal()

                if robot=="ur5":
                    gripper_goal.cmd_gripper.data = 0 # ROBOTIQ
                elif robot=="panda":
                    gripper_goal.cmd_gripper.data = 0.08 # PANDA GRIPPER
                    gripper_goal.flag.data = True # Use MoveAction instead of GraspAction

                return gripper_goal

            def place_result_cb(userdata, status, result):
                if status == GoalStatus.SUCCEEDED:
                    userdata.target_pose = result    
                    elapsed = (rospy.Time.now().to_sec() - userdata.handover_time)
                    rospy.loginfo("Robot maneuvering time: %.3f sec", elapsed)                
                    return 'succeeded'
                elif status == GoalStatus.PREEMPTED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('PLACE',
                                   smach_ros.SimpleActionState(
                                       'gripper_action_server',
                                       GripperAction,
                                       goal_cb=place_goal_cb,
                                       result_cb=place_result_cb,
                                       input_keys=['target_pose','handover_time'],
                                       output_keys=['target_pose','handover_time']),
                                   transitions={'succeeded': 'GO_BACK',
                                                'preempted': 'failed',
                                                'aborted': 'failed'},
                                    remapping={'target_pose':'target_pose','handover_time':'handover_time'})

                               
            def goback_goal_cb(userdata, goal):
                goback_goal = PoseGoal() 
                #EE displacement 
                goback_goal.goal_pose.pose.position.x =  0.01
                goback_goal.goal_pose.pose.position.y =  0.01
                goback_goal.goal_pose.pose.position.z = -0.09

                goback_goal.flag.data = True # It is used to transform the EE displacement in the world frame

                return goback_goal


            def goback_result_cb(userdata, status, result):
                if status == GoalStatus.SUCCEEDED:
                    if robot=="panda":
                        rospy.sleep(2)
                    return 'succeeded'
                elif status == GoalStatus.PREEMPTED:
                    return 'preempted'
                else:
                    return 'aborted'


            smach.StateMachine.add('GO_BACK',
                                   smach_ros.SimpleActionState(
                                       'pose_action_server',
                                       PoseAction,
                                       goal_cb=goback_goal_cb,
                                       result_cb=goback_result_cb,
                                       input_keys=['target_pose','handover_time'], 
                                       output_keys=['handover_time']),
                                       transitions={'succeeded': 'placed',
                                                'preempted': 'failed',
                                                'aborted': 'failed'})
                                                
        def rest_goal_cb(userdata, goal):
            rest_goal = JointGoal()
            rest_goal.rest.data = True               
            return rest_goal


        smach.StateMachine.add('REST',
                               smach_ros.SimpleActionState(
                                   'joint_action_server',
                                   JointAction,
                                   goal_cb=rest_goal_cb),
                               transitions={'succeeded': 'finished',
                                            'preempted': 'failed',
                                            'aborted': 'failed'})

        smach.StateMachine.add('HANDOVER', sm_handover,
                               transitions={'placed': 'SELECT',
                                            'failed': 'failed'})
        

    sis = smach_ros.IntrospectionServer('sm_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    print("Outcome: " + outcome)

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
