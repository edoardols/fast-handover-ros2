#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from filterpy.kalman import KalmanFilter
from collections import deque
import time
from std_msgs.msg import Header
import quaternion  

class SensorFusionKalmanNode(Node):
    def __init__(self):
        super().__init__("sensor_fusion_kalman_node")
        
        # Subscribers for two camera inputs
        self.sub_cam1 = self.create_subscription(PoseStamped, "realsense1_hand_detection", self.camera1_callback, 10)
        self.sub_cam2 = self.create_subscription(PoseStamped, "realsense2_hand_detection", self.camera2_callback, 10)
        
        # Publisher for fused pose
        self.fused_pub = self.create_publisher(PoseStamped, "hand_pose", 10)
        
        # Kalman Filter Initialization
        self.kf = KalmanFilter(dim_x=6, dim_z=3) # 6 states (x, y, z, vx, vy, vz) and 3 measurements (x, y, z)
        self.kf.F = np.array([[1, 0, 0, 1, 0, 0],
                               [0, 1, 0, 0, 1, 0],
                               [0, 0, 1, 0, 0, 1],
                               [0, 0, 0, 1, 0, 0],
                               [0, 0, 0, 0, 1, 0],
                               [0, 0, 0, 0, 0, 1]]) # State transition model
        
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                               [0, 1, 0, 0, 0, 0],
                               [0, 0, 1, 0, 0, 0]]) # Measurement function
        
        self.kf.P *= 1000 # Initial state covariance
        self.kf.R = np.eye(3) * 10 # Measurement noise covariance
        self.kf.Q = np.eye(6) * 0.01 # Process noise covariance

        # Exponential smoothing for orientation
        self.alpha = 0.05 # Smoothing factor
        self.prev_q = None
        
        self.get_logger().info("Kalman Filter Sensor Fusion Node Initialized.")
    
    def process_measurement(self, pose: PoseStamped):
        """Process a single pose measurement from any camera."""
        # -------------------- Position --------------------
        meas_pos = np.array([[pose.pose.position.x], [pose.pose.position.y], [pose.pose.position.z]])
        
        # Predict step
        self.kf.predict()
        # Update step with new measurement
        self.kf.update(meas_pos)
        # Get filtered state
        fused_pos = self.kf.x[:3]

        # -------------------- Orientation --------------------
        q_meas = np.quaternion(pose.pose.orientation.w,
                               pose.pose.orientation.x,
                               pose.pose.orientation.y,
                               pose.pose.orientation.z)

        if self.prev_q is None:
            q_fused = q_meas
        else:
            # Exponential quaternion smoothing via SLERP
            q_fused = quaternion.slerp_evaluate(self.prev_q, q_meas, self.alpha)

        self.prev_q = q_fused
        
        # Publish the fused pose
        fused_pose = PoseStamped()
        fused_pose.header.stamp = self.get_clock().now().to_msg()
        fused_pose.header.frame_id = "world"  
        fused_pose.pose.position.x = float(fused_pos[0, 0])
        fused_pose.pose.position.y = float(fused_pos[1, 0])
        fused_pose.pose.position.z = float(fused_pos[2, 0])
        fused_pose.pose.orientation.x = q_fused.x
        fused_pose.pose.orientation.y = q_fused.y
        fused_pose.pose.orientation.z = q_fused.z
        fused_pose.pose.orientation.w = q_fused.w

        self.fused_pub.publish(fused_pose)

    def camera1_callback(self, pose1: PoseStamped):
        self.process_measurement(pose1)
    
    def camera2_callback(self, pose2: PoseStamped):
        self.process_measurement(pose2)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionKalmanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()