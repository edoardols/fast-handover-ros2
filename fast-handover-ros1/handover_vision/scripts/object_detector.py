#!/usr/bin/env python3
from cv_bridge import CvBridge, CvBridgeError
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Transform, TransformStamped, Quaternion, Vector3Stamped, Point
from Segmentator_YOLOv8 import Segmentator
import quaternion
import math
from std_msgs.msg import String, Header
from utils import load_intrinsics, load_intrinsic_o3d, transform_pose
import open3d as o3d
from sensor_msgs import point_cloud2 as pc2f
from collections import deque
import tf
import sys 
import ast

class Extractor:

    def __init__(self):

        self.camera = sys.argv[1]

        self.rgb_topic = self.camera+rospy.get_param("/rgb_topic")
        self.depth_topic = self.camera+rospy.get_param("/depth_topic")
        self.camera_info_topic = self.camera+rospy.get_param("/camera_info_topic")

        frame = self.camera+"_color_optical_frame"

        listener = tf.TransformListener()
        listener.waitForTransform(frame, "world", rospy.Time(), rospy.Duration(4.0))
        (self.trans, self.rot) = listener.lookupTransform(frame, "world", rospy.Time(0))

        # Retrieve workspace limits
        self.xlim = ast.literal_eval(rospy.get_param('/xlim', '[0.2, 0.75]'))
        self.ylim = ast.literal_eval(rospy.get_param('/ylim', '[-0.35, 0.35]'))
        self.zlim = ast.literal_eval(rospy.get_param('/zlim', '[0.04, 0.5]'))
      
        self.bridge = CvBridge()
        self.width = 0
        self.height = 0
        
        self.model = Segmentator(0.25, 'yolov8m-seg.pt')
        
        # Subscribers
        camera_info_msg = rospy.wait_for_message(self.camera_info_topic, CameraInfo)
        self.intrinsics = load_intrinsic_o3d(camera_info_msg)

        self.imageRGB_sub = message_filters.Subscriber(self.rgb_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)

        self.ts = message_filters.TimeSynchronizer([self.imageRGB_sub, self.depth_sub], 1) 
        self.ts.registerCallback(self.callback)

        object_depth_img_topic = self.camera + "_object_depth_img"
        object_pc_topic = self.camera + "_object_pc" 
        debug_img_topic = self.camera + "_yolo_img" 

        # Publishers
        self.output_depth = rospy.Publisher(object_depth_img_topic, Image, queue_size=1)
        self.object_pc_pub = rospy.Publisher(object_pc_topic, PointCloud2, queue_size=1)

        self.debug = rospy.Publisher(debug_img_topic, Image, queue_size=1)

    def callback(self, imgRGB, depth_frame):


        header = imgRGB.header
        imgRGB = self.bridge.imgmsg_to_cv2(imgRGB)

        depth_frame = self.bridge.imgmsg_to_cv2(depth_frame)
        self.height = imgRGB.shape[0]
        self.width = imgRGB.shape[1]

        # segmentation to obtain object point cloud
        imgRGB, masks, classes = self.model.predict(imgRGB)

        # depth image is filtered to keep only object points
        final_depth = np.zeros(depth_frame.shape)

        obj_masks = list()
        remove_items = [56, 59, 60, 63, 8, 1, 0]                                                      # (chair, bed, dining table, laptop, bycicle, person)
        for i in range(len(classes)):
            if classes[i] not in remove_items:
                obj_masks.append(masks[:, :, i])

        # OR operation between object masks
        final_mask = np.zeros(depth_frame.shape)
        for mask in obj_masks:
            filter = mask * 1
            final_mask = np.logical_or(final_mask, filter)


                
        filtered_depth_frame = np.multiply(depth_frame, final_mask)                            # filtro la depth image per mantenere solo gli oggetti


        opcd = o3d.geometry.PointCloud()
        img = o3d.geometry.Image(filtered_depth_frame.astype(np.uint16))

        opcd = opcd.create_from_depth_image(img, intrinsic=self.intrinsics)


        # downsampling
        opcd = opcd.voxel_down_sample(voxel_size=0.005)

        opcd_table = self.crop_pc(opcd)
        pc, ind = opcd_table.remove_radius_outlier(nb_points=50, radius=0.02)
        inlier_cloud = opcd_table.select_by_index(ind)

        pc, ind = inlier_cloud.remove_radius_outlier(nb_points=300, radius=0.06)
        inlier_cloud = inlier_cloud.select_by_index(ind)

        opcd_table = inlier_cloud

        p = pc2f.create_cloud_xyz32(header, opcd_table.points)
        self.object_pc_pub.publish(p)


        img_msg_depth = self.bridge.cv2_to_imgmsg(filtered_depth_frame)
        img_msg_depth.header = header
        self.output_depth.publish(img_msg_depth)
        # debug image showing masks and bounding-box
        imgRGB = imgRGB.copy()
        img_msg = self.bridge.cv2_to_imgmsg(imgRGB, encoding='rgb8')
        self.debug.publish(img_msg)



    def crop_pc(self, pc):
        # prendi centro e orientamento della bb nel mondo e trasforma nel frame della camera
        table_pose = Pose()

        table_pose.position.x = (self.xlim[1]+self.xlim[0])/2
        table_pose.position.y = (self.ylim[1]+self.ylim[0])/2
        table_pose.position.z = (self.zlim[1]+self.zlim[0])/2

        table_pose.orientation.x = 0
        table_pose.orientation.y = 0
        table_pose.orientation.z = 0
        table_pose.orientation.w = 1
        table_pose = transform_pose(table_pose, self.trans, self.rot)
        header = Header()
        header.frame_id = self.camera + "_color_optical_frame"
        header.stamp = rospy.Time.now()
        
        center = np.array(([table_pose.position.x,
                            table_pose.position.y,
                            table_pose.position.z]))
        q = quaternion.as_quat_array(np.array(([table_pose.orientation.w,
                                                table_pose.orientation.x,
                                                table_pose.orientation.y,
                                                table_pose.orientation.z])))
        R = quaternion.as_rotation_matrix(q)
        extent = np.array([self.xlim[1]-self.xlim[0],self.ylim[1]-self.ylim[0],self.zlim[1]-self.zlim[0]])
        o3d_bbox = o3d.geometry.OrientedBoundingBox(center, R, extent)

        # self.table_pub.publish(header, table_pose)

        out_pc = pc.crop(o3d_bbox)
        return out_pc



if __name__ == '__main__':
    rospy.init_node('object_detector2')
    body = Extractor()
    rospy.spin()
