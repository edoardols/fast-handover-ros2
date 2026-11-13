import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_handover_camera = get_package_share_directory('handover_camera')

    camera_launch_file = os.path.join(pkg_handover_camera, 'launch', 'camera.launch.py')
    tags_launch_file = os.path.join(pkg_handover_camera, 'launch', 'tags.launch.py')

    rviz_config_file = os.path.join(pkg_handover_camera, 'rviz', 'camera_debug.rviz')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file),
        launch_arguments={'camera_name': 'realsense1'}.items(),
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

    return LaunchDescription(
        [
            camera_launch,
            tags_launch,
            rviz_node,
        ]
    )
