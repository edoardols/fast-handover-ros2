#!/usr/bin/env python3
import math
from cv_bridge import CvBridge
import rospy
import mediapipe as mp
import numpy as np
# from collections import deque
from sensor_msgs import point_cloud2 as pc2f
from std_msgs.msg import String, Float64
import cv2
import message_filters
from utils import load_intrinsics, transform_pose
from sensor_msgs.msg import Image, CameraInfo
import pyrealsense2 as rs
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, PoseStamped
import tf
import sys 
import ast
import quaternion

class HandEstimation:

    def __init__(self):
        self.camera = sys.argv[1]
        rgb_topic = self.camera+rospy.get_param("/rgb_topic")
        depth_topic = self.camera+rospy.get_param("/depth_topic")
        camera_info_topic = self.camera+rospy.get_param("/camera_info_topic")
        camera_link = self.camera + "_color_optical_frame"

        # Retrieve workspace limits
        self.xlim = ast.literal_eval(rospy.get_param('/xlim', '[0.2, 0.75]'))
        self.ylim = ast.literal_eval(rospy.get_param('/ylim', '[-0.35, 0.35]'))
        self.zlim = ast.literal_eval(rospy.get_param('/zlim', '[0.04, 0.5]'))

        self.bridge = CvBridge()

        listener = tf.TransformListener()
        listener.waitForTransform("world", camera_link, rospy.Time(), rospy.Duration(4.0))

        (self.trans, self.rot) = listener.lookupTransform("world", camera_link, rospy.Time(0))
                         
        
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(model_complexity=1,
                                         min_detection_confidence = 0.6, min_tracking_confidence = 0.3, max_num_hands = 2)

        # Subscribers

        camera_info_msg = rospy.wait_for_message(camera_info_topic, CameraInfo)
        self.intrinsics = load_intrinsics(camera_info_msg)        

        self.img_reader = message_filters.Subscriber(rgb_topic, Image)
        self.depth_reader = message_filters.Subscriber(depth_topic, Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.img_reader, self.depth_reader], 1, slop = 0.05) 
        self.ts.registerCallback(self.run_inference)


        # Publishers
        self.all_hand_poses_pub = rospy.Publisher(self.camera+'_all_hand_poses', PoseArray, queue_size=1)    
        self.hand_pose_pub = rospy.Publisher(self.camera+'_hand_detection', PoseStamped, queue_size=1)    

        # if self.debug_mode:
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_pub = rospy.Publisher(self.camera+'_mediapipe_output', Image, queue_size=1)    


    def run_inference(self, rgb_img, depth_img):

        header = rgb_img.header

        hand_poses = self.get_hand_poses(rgb_img, depth_img, 1)      # returns hand poses in world frame, excluding those outside the workspace

        if len(hand_poses) != 0:
            
            all_hand_msg = PoseArray()
            for pose in hand_poses: 
                all_hand_msg.poses.append(pose)      
            all_hand_msg.header = header
            all_hand_msg.header.frame_id = "world"
            self.all_hand_poses_pub.publish(all_hand_msg)

            closest_hand = self.detect_closest_hand(all_hand_msg.poses)
            self.hand_pose_pub.publish(PoseStamped(all_hand_msg.header, closest_hand))

    


    def get_hand_poses(self, rgb_img, depth_img, camera_id):
        img_cv = self.bridge.imgmsg_to_cv2(rgb_img, "rgb8")
        results = self.hands.process(img_cv)

        self.height = img_cv.shape[0]
        self.width = img_cv.shape[1]
        
        depth_frame = self.bridge.imgmsg_to_cv2(depth_img)
        intrinsics = self.intrinsics

        hand_poses = []

        if results.multi_hand_landmarks is not None:    
            for i in range(len(results.multi_hand_landmarks)):   # for each hand
                kp_positions = []
                indexes = [0, 5, 9, 13, 17]     # keypoints used for hand pose estimation
                for j in range(len(indexes)):
                    x = [results.multi_hand_landmarks[i].landmark[indexes[j]].x]
                    y = [results.multi_hand_landmarks[i].landmark[indexes[j]].y]
                    x, y = self.adjust_kp_xy(x[0]*self.width, y[0]*self.height)

                    depth = depth_frame[y, x]
                    if depth != 0:
                        dx, dy, dz = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth) 
                        kp_positions.append(np.array(([dx/1000, dy/1000, dz/1000])).T)



                if len(kp_positions) == len(indexes):
                    kp_positions = np.array(kp_positions)

                    centroid = np.mean(kp_positions, 0)

                    # Define x_dir as direction from wrist (0) to index base (5)
                    x_dir = kp_positions[2] - kp_positions[0] 
                    x_dir /= np.linalg.norm(x_dir)

                    # Define z_dir as palm normal: cross product of (middle base - wrist) and (ring base - wrist)
                    v1 = kp_positions[1] - kp_positions[0]  # 9 - 0
                    v2 = kp_positions[4] - kp_positions[0]  # 13 - 0
                    z_dir = np.cross(v1, v2)

                    if np.linalg.norm(z_dir) < 1e-6:
                        # Degenerate case: skip this hand 
                        continue  
                    else:
                        z_dir /= np.linalg.norm(z_dir)

                    # Define y_dir as orthogonal to z and x 
                    y_dir = np.cross(z_dir, x_dir)
                    y_dir /= np.linalg.norm(y_dir)

                    # Form rotation matrix with column stacking
                    R = np.column_stack([x_dir, y_dir, z_dir])


                    centroid_pos = Pose(position=Point(*centroid), orientation=quaternion.from_rotation_matrix(R))
                    centroid_pos = transform_pose(centroid_pos, self.trans, self.rot)
                   
                    # adjust hand orientation such as x is pointing towards the robot
                    centroid_pos = self.adjust_hand_pose(centroid_pos)  

                    if  self.xlim[0] <= centroid_pos.position.x <= self.xlim[1] and self.ylim[0] <= centroid_pos.position.y <= self.ylim[1]  and self.zlim[0] <= centroid_pos.position.z <= self.zlim[1] :       # da aggiustare in base a word 
                        hand_poses.append(centroid_pos) 
                    
            img_cv = cv2.cvtColor(img_cv, cv2.COLOR_RGB2BGR)
            for i in range(0, len(results.multi_hand_landmarks)):
                self.mp_drawing.draw_landmarks(
                    img_cv, results.multi_hand_landmarks[i], self.mp_hands.HAND_CONNECTIONS)
            img_ros = self.bridge.cv2_to_imgmsg(img_cv, "bgr8")
            self.mp_pub.publish(img_ros)
               
        return hand_poses
        



    def detect_closest_hand(self, hand_list):  
        closest_hand = None
        closest_distance = float('inf')
        
        for hand in hand_list:
            # Calculate position module (magnitude)
            position_module = np.linalg.norm(np.array(([hand.position.x, hand.position.y, hand.position.z])))
            
            # Check if this hand is closer than the previously closest hand
            if position_module < closest_distance:
                closest_hand = hand
                closest_distance = position_module
        return closest_hand
    

    def adjust_kp_xy(self, x, y):
        # if the keypoint is outside the image the nearest border pixel is considered
        x_f = min(int(x), self.width - 1)
        y_f = min(int(y), self.height - 1)
        return x_f, y_f

    def adjust_hand_pose(self, pose):
        q = quaternion.as_quat_array(np.array(([
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z])))
        rot = quaternion.as_rotation_matrix(q)
        x_proj = rot[0, 0]
        if x_proj > 0:
            rotator = np.matrix([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
            rot = np.dot(rotator, rot)  
            pose.orientation = quaternion.from_rotation_matrix(rot)
        return pose



if __name__ == '__main__':
    rospy.init_node('hand_detector')
    body = HandEstimation()
    rospy.spin()
