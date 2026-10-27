#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        self.sub_raw_pose = self.create_subscription(PoseStamped, '/hand_pose', self.pose_cb, 10)
        self.pub_filtered_pose = self.create_publisher(PoseStamped, '/hand_pose_filtered', 10)

        # Stato: [x, y, z, vx, vy, vz]
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 1.0
        self.F = np.eye(6)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0

        self.R = np.eye(3) * 0.01
        self.Q = np.eye(6) * 0.001

        self.last_time = self.get_clock().now()
        self.initialized = False

    def pose_cb(self, msg: PoseStamped):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0:
            dt = 0.033

        z = np.array([[msg.pose.position.x], [msg.pose.position.y], [msg.pose.position.z]])

        if not self.initialized:
            self.x[:3] = z
            self.initialized = True
            return

        # Predict
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Update
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y)
        self.P = (np.eye(6) - (K @ self.H)) @ self.P

        filtered_msg = PoseStamped()
        filtered_msg.header = msg.header
        filtered_msg.pose.position.x = float(self.x[0, 0])
        filtered_msg.pose.position.y = float(self.x[1, 0])
        filtered_msg.pose.position.z = float(self.x[2, 0])
        filtered_msg.pose.orientation = msg.pose.orientation

        self.pub_filtered_pose.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()