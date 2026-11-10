import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_mtc_tutorial = get_package_share_directory('mtc_tutorial')
    pkg_realsense = get_package_share_directory('realsense2_camera')

    tags_config_path = os.path.join(pkg_mtc_tutorial, 'config', 'tags.yaml')

    # 1. Driver RealSense D435
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'rgb_camera.profile': '1280x720x30',
            'enable_color': 'true',
            'enable_depth': 'true',
        }.items()
    )

    # 2. Nodo AprilTag
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[tags_config_path],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ]
    )

    return LaunchDescription([
        realsense_launch,
        apriltag_node,
    ])