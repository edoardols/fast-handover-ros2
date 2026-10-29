#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import PointCloud2
import open3d as o3d
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox3D
from src.handover_vision.handover_vision.utils import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Transform, Quaternion, Point

class PointCloudCombiner(Node):
    def __init__(self):
        super().__init__('point_cloud_combiner')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.trans_camera1 = None
        self.trans_camera2 = None

        pc1_table_sub = message_filters.Subscriber(self, PointCloud2, '/realsense1_object_pc')
        pc2_table_sub = message_filters.Subscriber(self, PointCloud2, '/realsense2_object_pc')
        
        self.ts_table = message_filters.ApproximateTimeSynchronizer([pc1_table_sub, pc2_table_sub], queue_size=10, slop=0.1)
        self.ts_table.registerCallback(self.callback)
        
        self.pc_combined_pub = self.create_publisher(PointCloud2, '/object_pointcloud', 10)
        self.table_pc_combined_pub = self.create_publisher(PointCloud2, '/table_object_pointcloud', 10)
        self.object_pose_pub = self.create_publisher(PoseStamped, '/object_pose', 1)
        self.bb_pub = self.create_publisher(BoundingBox3D, '/object_bb', 1)

    def callback(self, pc1, pc2):
        try:
            self.trans_camera1 = self.tf_buffer.lookup_transform("world", "realsense1_color_optical_frame", rclpy.time.Time())
            #  self.trans_camera1 = tf_buffer.lookup_transform("world", "realsense1_color_optical_frame",
            #                                          rospy.Time(0),
            #                                          rospy.Duration(10))
            self.trans_camera2 = self.tf_buffer.lookup_transform("world", "realsense2_color_optical_frame", rclpy.time.Time())
            #  self.trans_camera2 = tf_buffer.lookup_transform("world", "realsense2_color_optical_frame",
            #                                          rospy.Time(0),
            #                                          rospy.Duration(10))
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Waiting for transforms: {ex}")
            return

        header = pc1.header
        pc1_tf = do_transform_cloud(pc1, self.trans_camera1)
        pc2_tf = do_transform_cloud(pc2, self.trans_camera2)

        pcd_combined = o3d.geometry.PointCloud()

        pc1_o3d = convertCloudFromRosToOpen3d(pc1_tf)
        pc2_o3d = convertCloudFromRosToOpen3d(pc2_tf)
        
        if pc1_o3d is not None:
            pcd_combined = pcd_combined + pc1_o3d 
        if pc2_o3d is not None:
            pcd_combined = pcd_combined + pc2_o3d 

        if len(pcd_combined.points) > 10:
            # compute and publish bb
            bb = pcd_combined.get_axis_aligned_bounding_box()
            center = bb.get_center()
            extent = bb.get_extent()
            
            bb_msg = BoundingBox3D()
            # bb_msg = BoundingBox3D(Pose(position=Point(*center), orientation=Quaternion(0, 0, 0, 1)), Vector3(*extent))
            bb_msg.center = Pose(position=Point(x=center[0], y=center[1], z=center[2]), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
            bb_msg.size = Vector3(x=extent[0], y=extent[1], z=extent[2])
            
            self.bb_pub.publish(bb_msg)
            
            header.frame_id = "world"
            
            ps = PoseStamped()
            ps.header = header
            # self.object_pose_pub.publish(PoseStamped(header, Pose(position=Point(*center), orientation=Quaternion(0, 0, 0, 1))))
            ps.pose.position = Point(x=center[0], y=center[1], z=center[2])
            ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            self.object_pose_pub.publish(ps)
            
            combined_pc = convertCloudFromOpen3dToRos(pcd_combined, header)
            combined_pc.header.frame_id = "world" 
            
            self.table_pc_combined_pub.publish(combined_pc)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCombiner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()