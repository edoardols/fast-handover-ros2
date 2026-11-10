import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# from handover_camera.launch_utils import get_static_tf_REP103_OpenCV

def generate_launch_description():
    pkg_realsense = get_package_share_directory('realsense2_camera')
    pkg_handover_vision = get_package_share_directory('handover_vision')
    
    rviz_config_file = os.path.join(pkg_handover_vision, 'rviz', 'vision.rviz')

    # Driver Intel RealSense Depth Camera D435
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            # 'rgb_camera.color_profile': '1280x720x30', # High precision
            'rgb_camera.color_profile': '640x480x15', # Low precision
            'enable_color': 'true',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'pointcloud.enable': 'false',
            'enable_sync': 'true',
        }.items()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[],
    )

    hand_detector_node = Node(
        package="handover_vision",
        executable="hand_detector",
        output="screen",
        parameters=[],
    )

    object_detector_node = Node(
        package="handover_vision",
        executable="object_detector",
        output="screen",
        parameters=[],
    )

    kalman_filter_node = Node(
        package="handover_vision",
        executable="kalman_filter_node",
        output="screen",
        parameters=[],
    )

    pointcloud_merger_node = Node(
        package="handover_vision",
        executable="pointcloud_merger",
        output="screen",
        parameters=[],
    )

    return LaunchDescription(
        [
            realsense_launch,
            # anchor_tf,
            # static_tf_tag_base_to_optical,
            # List of all static TFs for detected AprilTags
            # *static_tf_tags,
            # delayed_vision_nodes,
            # apriltag_node,
            # camera_tf_node,
            rviz_node,
            hand_detector_node,
            # object_detector_node,
            # kalman_filter_node,
            # pointcloud_merger_node,
        ]
    )
