#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import BoundingBox3D
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        self.bridge = CvBridge()

        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        
        self.pub_obj_pose = self.create_publisher(PoseStamped, '/object_pose', 10)
        self.pub_obj_bb = self.create_publisher(BoundingBox3D, '/object_bb', 10)

        self.depth_image = None

    def depth_cb(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Errore cv_bridge depth: {e}")

    def image_cb(self, msg: Image):
        if self.depth_image is None:
            return

        # Pubblica valori di bounding box ed enucleazione di test
        bb_msg = BoundingBox3D()
        bb_msg.center.position.x = 0.4
        bb_msg.center.position.y = 0.0
        bb_msg.center.position.z = 0.5
        bb_msg.size.x = 0.08
        bb_msg.size.y = 0.08
        bb_msg.size.z = 0.15
        self.pub_obj_bb.publish(bb_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = msg.header.frame_id
        pose_msg.pose.position.x = 0.4
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.5
        pose_msg.pose.orientation.w = 1.0
        self.pub_obj_pose.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()