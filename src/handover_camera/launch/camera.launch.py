import os
from sympy import true
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_node(context, *args, **kwargs):
    pkg_handover_camera = get_package_share_directory('handover_camera')
    pkg_realsense = get_package_share_directory('realsense2_camera')
    
    tags_config_file = os.path.join(pkg_handover_camera, 'config', 'tags.yaml')

    camera_name = LaunchConfiguration('camera_name').perform(context)

    # Driver Intel RealSense Depth Camera D435
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_name': camera_name,
            'rgb_camera.color_profile': '640x480x15', # Low precision
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'pointcloud.enable': 'true',
            'enable_sync': 'true',
            # 'initial_reset': 'true',
            'align_depth.enable': 'true',
            'depth_module.depth_profile': '640x480x15'
        }.items()
    )
    
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name=f'apriltag_node_{camera_name}',
        output='screen',
        parameters=[
            tags_config_file,
            {f'qos_overrides./camera/{camera_name}/color/camera_info.subscription.reliability': 'best_effort'},
            {f'qos_overrides./camera/{camera_name}/color/image_raw.subscription.reliability': 'best_effort'},
        ],
        remappings=[
            ('image_rect', f'/camera/{camera_name}/color/image_raw'),
            ('camera_info', f'/camera/{camera_name}/color/camera_info')
        ]
    )

    camera_tf_node = Node(
        package='handover_camera',
        executable='camera_tf_broadcaster',
        name=f'camera_tf_node_{camera_name}',
        output='screen',
        parameters=[
            {'camera_name_param': camera_name}
        ]
    )

    return ([   
        realsense_node,
        apriltag_node,
        camera_tf_node,
    ])

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='realsense1'),
        OpaqueFunction(function=launch_node)
    ])
