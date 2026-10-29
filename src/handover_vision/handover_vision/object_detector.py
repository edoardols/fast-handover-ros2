#!/usr/bin/env python3
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Transform, TransformStamped, Quaternion, Vector3Stamped, Point
from src.handover_vision.handover_vision.Segmentator_YOLOv8 import Segmentator
import quaternion
import math
from std_msgs.msg import String, Header
from src.handover_vision.handover_vision.utils import load_intrinsics, load_intrinsic_o3d, transform_pose
import open3d as o3d
from sensor_msgs import point_cloud2 as pc2f
from collections import deque
import tf2_ros
import sys 
import ast

class Extractor(Node):

    def __init__(self):
        super().__init__('object_detector2')

        self.declare_parameter('camera_name', 'camera')
        camera_arg = self.get_parameter('camera_name').value
        if len(sys.argv) > 1 and not sys.argv[1].startswith('__'):
             self.camera = sys.argv[1]
        else:
             self.camera = camera_arg

        self.declare_parameter("/rgb_topic", "/color/image_raw")
        self.declare_parameter("/depth_topic", "/aligned_depth_to_color/image_raw")
        self.declare_parameter("/camera_info_topic", "/color/camera_info")

        self.rgb_topic = self.camera + self.get_parameter("/rgb_topic").value
        self.depth_topic = self.camera + self.get_parameter("/depth_topic").value
        self.camera_info_topic = self.camera + self.get_parameter("/camera_info_topic").value

        self.frame = self.camera+"_color_optical_frame"

        self.declare_parameter('/xlim', '[0.2, 0.75]')
        self.declare_parameter('/ylim', '[-0.35, 0.35]')
        self.declare_parameter('/zlim', '[0.04, 0.5]')

        # [LEGACY ROS 1]
        # listener = tf.TransformListener()
        # listener.waitForTransform(frame, "world", rospy.Time(), rospy.Duration(4.0))
        # (self.trans, self.rot) = listener.lookupTransform(frame, "world", rospy.Time(0))
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.trans = [0.0, 0.0, 0.0]
        self.rot = [0.0, 0.0, 0.0, 1.0]
        self.tf_ready = False

        # Retrieve workspace limits
        self.xlim = ast.literal_eval(self.get_parameter('/xlim').value)
        self.ylim = ast.literal_eval(self.get_parameter('/ylim').value)
        self.zlim = ast.literal_eval(self.get_parameter('/zlim').value)
      
        self.bridge = CvBridge()
        self.width = 0
        self.height = 0
        
        self.model = Segmentator(0.25, 'yolov8m-seg.pt')
        
        self.intrinsics = None
        self.cam_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 1)

        self.imageRGB_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)

        self.ts = message_filters.TimeSynchronizer([self.imageRGB_sub, self.depth_sub], 1) 
        self.ts.registerCallback(self.callback)

        object_depth_img_topic = self.camera + "_object_depth_img"
        object_pc_topic = self.camera + "_object_pc" 
        debug_img_topic = self.camera + "_yolo_img" 

        # Publishers
        self.output_depth = self.create_publisher(Image, object_depth_img_topic, 1)
        self.object_pc_pub = self.create_publisher(PointCloud2, object_pc_topic, 1)
        self.debug = self.create_publisher(Image, debug_img_topic, 1)

    def camera_info_callback(self, msg):
        if self.intrinsics is None:
            # self.intrinsics = load_intrinsic_o3d(camera_info_msg)
            self.intrinsics = load_intrinsic_o3d(msg)

    def callback(self, imgRGB, depth_frame):
        if self.intrinsics is None:
             return
             
        if not self.tf_ready:
             try:
                 t = self.tf_buffer.lookup_transform(self.frame, "world", rclpy.time.Time())
                 self.trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
                 self.rot = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
                 self.tf_ready = True
             except tf2_ros.TransformException as ex:
                 self.get_logger().warn(f"Waiting for transform: {ex}")
                 return

        header = imgRGB.header
        imgRGB_cv = self.bridge.imgmsg_to_cv2(imgRGB)
        depth_frame_cv = self.bridge.imgmsg_to_cv2(depth_frame)
        self.height = imgRGB_cv.shape[0]
        self.width = imgRGB_cv.shape[1]

        # segmentation to obtain object point cloud
        imgRGB_res, masks, classes = self.model.predict(imgRGB_cv)

        # depth image is filtered to keep only object points
        final_depth = np.zeros(depth_frame_cv.shape)

        obj_masks = list()
        remove_items = [56, 59, 60, 63, 8, 1, 0]                                                      
        for i in range(len(classes)):
            if classes[i] not in remove_items:
                if len(masks.shape) > 2:
                    obj_masks.append(masks[:, :, i])
                else:
                    obj_masks.append(masks)

        # OR operation between object masks
        final_mask = np.zeros(depth_frame_cv.shape)
        for mask in obj_masks:
            filter_m = mask * 1
            final_mask = np.logical_or(final_mask, filter_m)
                
        filtered_depth_frame = np.multiply(depth_frame_cv, final_mask)                            

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

        img_msg_depth = self.bridge.cv2_to_imgmsg(filtered_depth_frame.astype(np.uint16))
        img_msg_depth.header = header
        self.output_depth.publish(img_msg_depth)

        # debug image showing masks and bounding-box
        imgRGB_res_cpy = imgRGB_res.copy()
        img_msg = self.bridge.cv2_to_imgmsg(imgRGB_res_cpy, encoding='rgb8')
        img_msg.header = header
        self.debug.publish(img_msg)

    def crop_pc(self, pc):
        # prendi centro e orientamento della bb nel mondo e trasforma nel frame della camera
        table_pose = Pose()

        table_pose.position.x = (self.xlim[1]+self.xlim[0])/2
        table_pose.position.y = (self.ylim[1]+self.ylim[0])/2
        table_pose.position.z = (self.zlim[1]+self.zlim[0])/2

        table_pose.orientation.x = 0.0
        table_pose.orientation.y = 0.0
        table_pose.orientation.z = 0.0
        table_pose.orientation.w = 1.0
        
        table_pose = transform_pose(table_pose, self.trans, self.rot)
        
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

        # [LEGACY ROS 1 not implemented]
        # header = Header()
        # header.frame_id = self.camera + "_color_optical_frame"
        # header.stamp = rospy.Time.now()
        # self.table_pub.publish(header, table_pose)

        out_pc = pc.crop(o3d_bbox)
        return out_pc

def main(args=None):
    rclpy.init(args=args)
    node = Extractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()