#!/usr/bin/env python3

import rospy
import message_filters
from sensor_msgs.msg import PointCloud2
import open3d as o3d
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox3D
from utils import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Transform, Quaternion, Point

class PointCloudCombiner:
    def __init__(self):

        rospy.init_node('point_cloud_combiner', anonymous=True)

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        self.trans_camera1 = tf_buffer.lookup_transform("world", "realsense1_color_optical_frame",
                                        rospy.Time(0),
                                        rospy.Duration(10))
        self.trans_camera2 = tf_buffer.lookup_transform("world", "realsense2_color_optical_frame",
                                        rospy.Time(0),
                                        rospy.Duration(10))


        pc1_table_sub = message_filters.Subscriber('/realsense1_object_pc', PointCloud2)
        pc2_table_sub = message_filters.Subscriber('/realsense2_object_pc', PointCloud2)
        
        self.ts_table = message_filters.ApproximateTimeSynchronizer([pc1_table_sub, pc2_table_sub], queue_size=10, slop=0.1)
        self.ts_table.registerCallback(self.callback)
        
        self.pc_combined_pub = rospy.Publisher('/object_pointcloud', PointCloud2, queue_size=10)
        self.table_pc_combined_pub = rospy.Publisher('/table_object_pointcloud', PointCloud2, queue_size=10)
        self.object_pose_pub = rospy.Publisher('/object_pose', PoseStamped, queue_size=1)
        self.bb_pub = rospy.Publisher('/object_bb', BoundingBox3D, queue_size=1)

    
    def callback(self, pc1, pc2):

        header = pc1.header
        pc1 = do_transform_cloud(pc1, self.trans_camera1)
        pc2 = do_transform_cloud(pc2, self.trans_camera2)

        pcd_combined = o3d.geometry.PointCloud()

        pc1_o3d = convertCloudFromRosToOpen3d(pc1)
        pc2_o3d = convertCloudFromRosToOpen3d(pc2)
        
        if pc1_o3d is not None:
            pcd_combined = pcd_combined + pc1_o3d 
        if pc2_o3d is not None:
            pcd_combined = pcd_combined + pc2_o3d 

        if len(pcd_combined.points) > 10:
            # compute and publish bb
            bb = pcd_combined.get_axis_aligned_bounding_box()
            center = bb.get_center()
            extent = bb.get_extent()
            bb_msg = BoundingBox3D(Pose(position=Point(*center), orientation=Quaternion(0, 0, 0, 1)), Vector3(*extent))
            self.bb_pub.publish(bb_msg)
            
            header.frame_id = "world"
            self.object_pose_pub.publish(PoseStamped(header, Pose(position=Point(*center), orientation=Quaternion(0, 0, 0, 1))))
      
            
            combined_pc = convertCloudFromOpen3dToRos(pcd_combined, header)
            combined_pc.header.frame_id = "world" 
            
            self.table_pc_combined_pub.publish(combined_pc)

            

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        combiner = PointCloudCombiner()
        combiner.run()
    except rospy.ROSInterruptException:
        pass
