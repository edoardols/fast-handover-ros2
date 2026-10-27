#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointcloudMergerNode(Node):
    def __init__(self):
        super().__init__('pointcloud_merger_node')

        self.sub_pc1 = self.create_subscription(PointCloud2, '/camera1/depth/color/points', self.pc1_cb, 10)
        self.sub_pc2 = self.create_subscription(PointCloud2, '/camera2/depth/color/points', self.pc2_cb, 10)
        self.pub_merged_pc = self.create_publisher(PointCloud2, '/merged_pointcloud', 10)

    def pc1_cb(self, msg: PointCloud2):
        self.pub_merged_pc.publish(msg)

    def pc2_cb(self, msg: PointCloud2):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PointcloudMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()