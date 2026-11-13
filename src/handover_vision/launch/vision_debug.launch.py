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
    pkg_handover_vision = get_package_share_directory('handover_vision')

    pkg_handover_camera = get_package_share_directory('handover_camera')
    camera_launch_file = os.path.join(pkg_handover_camera, 'launch', 'camera.launch.py')
    tags_launch_file = os.path.join(pkg_handover_camera, 'launch', 'tags.launch.py')
    
    rviz_config_file = os.path.join(pkg_handover_vision, 'rviz', 'vision_debug.rviz')

    camera_name1 = "realsense1"
    camera_name2 = "camera"
    xlim = '[-1.0, 1.0]'
    ylim = '[-1.0, 1.0]'
    zlim = '[-1.0, 1.0]'

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file),
        launch_arguments={'camera_name': camera_name1}.items(),
    )

    tags_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tags_launch_file),
        launch_arguments={},
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[],
    )

    hand_detector_node1 = Node(
        package="handover_vision",
        executable="hand_detector.py",
        name=f"hand_detector_{camera_name1}",
        output="screen",
        parameters=[
            {'camera_name': camera_name1},
            {'/xlim': xlim},
            {'/ylim': ylim},
            {'/zlim': zlim},
            ],
    )

    object_detector_node1 = Node(
        package="handover_vision",
        executable="object_detector.py",
        name=f"object_detector_node_{camera_name1}",
        output="screen",
        parameters=[
            {'camera_name': camera_name1},
            {'/xlim': xlim},
            {'/ylim': ylim},
            {'/zlim': zlim},
        ],
    )

    kalman_filter_node = Node(
        package="handover_vision",
        executable="kalman_filter.py",
        name="kalman_filter_node",
        output="screen",
        parameters=[
            {'camera_name1': camera_name1},
            {'camera_name2': camera_name2},
            {'/xlim': xlim},
            {'/ylim': ylim},
            {'/zlim': zlim},
        ],
    )

    pointcloud_merger_node = Node(
        package="handover_vision",
        executable="pointcloud_merger.py",
        name="pointcloud_merger_node",
        output="screen",
        parameters=[
            {'camera_name1': camera_name1},
            {'camera_name2': camera_name2},
            {'/xlim': xlim},
            {'/ylim': ylim},
            {'/zlim': zlim},
        ],
    )

    return LaunchDescription(
        [
            # camera_launch,
            tags_launch,
            hand_detector_node1,
            object_detector_node1,
            kalman_filter_node,
            pointcloud_merger_node,
            rviz_node,
        ]
    )
