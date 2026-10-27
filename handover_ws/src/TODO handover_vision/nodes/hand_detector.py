#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    import mediapipe as mp
except ImportError:
    mp = None

class HandDetectorNode(Node):
    def __init__(self):
        super().__init__('hand_detector_node')
        
        if mp is None:
            self.get_logger().error("MediaPipe non installato. Installa con: pip install mediapipe")

        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6
        ) if mp else None

        self.sub_rgb = self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_cb, 10)
        self.sub_cam_info = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_cb, 10)

        self.pub_hand_pose = self.create_publisher(PoseStamped, '/hand_pose', 10)

        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

    def info_cb(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]

    def depth_cb(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Errore cv_bridge depth: {e}")

    def image_cb(self, msg: Image):
        if self.mp_hands is None or self.depth_image is None or self.fx is None:
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Errore cv_bridge RGB: {e}")
            return

        image_rgb = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        results = self.mp_hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                wrist = hand_landmarks.landmark[0]
                h, w, _ = rgb_image.shape
                u, v = int(wrist.x * w), int(wrist.y * h)

                if 0 <= u < w and 0 <= v < h:
                    depth = self.depth_image[v, u] / 1000.0  # mm to meters
                    if depth > 0:
                        x_3d = (u - self.cx) * depth / self.fx
                        y_3d = (v - self.cy) * depth / self.fy
                        z_3d = depth

                        pose_msg = PoseStamped()
                        pose_msg.header.stamp = self.get_clock().now().to_msg()
                        pose_msg.header.frame_id = msg.header.frame_id
                        pose_msg.pose.position.x = x_3d
                        pose_msg.pose.position.y = y_3d
                        pose_msg.pose.position.z = z_3d
                        pose_msg.pose.orientation.w = 1.0

                        self.pub_hand_pose.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()