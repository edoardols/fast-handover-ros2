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
    
    rviz_config_file = os.path.join(pkg_handover_vision, 'rviz', 'vision_debug.rviz')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file),
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

    hand_detector_node = Node(
        package="handover_vision",
        executable="hand_detector.py",
        name="hand_detector1",
        output="screen",
        parameters=['realsense1'],
    )

    object_detector_node = Node(
        package="handover_vision",
        executable="object_detector.py",
        output="screen",
        parameters=[],
    )

    kalman_filter_node = Node(
        package="handover_vision",
        executable="kalman_filter.py",
        output="screen",
        parameters=[],
    )

    pointcloud_merger_node = Node(
        package="handover_vision",
        executable="pointcloud_merger.py",
        output="screen",
        parameters=[],
    )

    return LaunchDescription(
        [
            camera_launch,
            hand_detector_node,
            object_detector_node,
            kalman_filter_node,
            pointcloud_merger_node,
            rviz_node,
        ]
    )
