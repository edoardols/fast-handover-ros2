#!/usr/bin/env python3

import pyrealsense2 as rs
import open3d 
from geometry_msgs.msg import Pose
import numpy as np 
from tf.transformations import quaternion_matrix
import rospy
import sensor_msgs.point_cloud2 as pc2
import quaternion
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
                [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2 ** 16
BIT_MOVE_8 = 2 ** 8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000) >> 16, (rgb_uint32 & 0x0000ff00) >> 8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)


def load_intrinsics(camera_info_msg):
    intrinsics = rs.intrinsics()
    intrinsics.width = camera_info_msg.width
    intrinsics.height = camera_info_msg.height
    intrinsics.ppx = camera_info_msg.K[2]
    intrinsics.ppy = camera_info_msg.K[5]
    intrinsics.fx = camera_info_msg.K[0]
    intrinsics.fy = camera_info_msg.K[4]
    intrinsics.model = rs.distortion.none
    intrinsics.coeffs = [i for i in camera_info_msg.D]
    return intrinsics

def load_intrinsic_o3d(camera_info_msg):
    width = camera_info_msg.width
    height = camera_info_msg.height
    ppx = camera_info_msg.K[2]
    ppy = camera_info_msg.K[5]
    fx = camera_info_msg.K[0]
    fy = camera_info_msg.K[4]
    return open3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, ppx, ppy)


def transform_pose(pose, trans, rot):
    # Convert quaternion to transformation matrix
    trans_matrix = quaternion_matrix(rot)
    # Apply translation
    trans_matrix[:3, 3] = trans
    # trans_matrix[:3, :3] = np.transpose(trans_matrix[:3, :3])

    # Apply transformation to pose
    new_pose = Pose()
    # new_pose.header.frame_id = "/world"  # Replace with your desired frame
    # new_pose.header.stamp = rospy.Time.now()

    # Apply transformation to position
    # new_position = trans_matrix.dot([pose.pose.position.x,
    #                                 pose.pose.position.y,
    #                                 pose.pose.position.z,
    #                                 1.0])[:3]
    
    old_position = np.array(([pose.position.x, pose.position.y, pose.position.z, 1])).reshape((4, 1))
    new_position = np.dot(trans_matrix, old_position)
    new_pose.position.x, new_pose.position.y, new_pose.position.z = new_position[0:3]

    # Apply transformation to orientation
    # Note: Assuming orientation is in quaternion format

    q = quaternion.as_quat_array(np.array(([
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z])))
    R = quaternion.as_rotation_matrix(q)
    R_new = np.dot(trans_matrix[:3, :3], R)
    new_pose.orientation = quaternion.from_rotation_matrix(R_new)

    return new_pose



# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points = np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors:  # XYZ only
        fields = FIELDS_XYZ
        cloud_data = points
    else:  # XYZ + RGB
        fields = FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors) * 255)  # nx3 matrix
        colors = colors[:, 0] * BIT_MOVE_16 + colors[:, 1] * BIT_MOVE_8 + colors[:, 2]
        cloud_data = np.c_[points, colors]

    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)


def convertCloudFromRosToOpen3d(ros_cloud):
    # Get cloud data from ros_cloud
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))

    # Check empty
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data) == 0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD = 3  # x, y, z, rgb

        # Get xyz
        xyz = [(x, y, z) for x, y, z, rgb in cloud_data]  # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD]) == float:  # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x, y, z, rgb in cloud_data]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x, y, z, rgb in cloud_data]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb) / 255.0)
    else:
        xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud
