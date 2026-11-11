import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def get_static_tf_REP103_OpenCV(tag_REP103="tag_REP103", tag_OpenCV="tag_OpenCV", REP103_to_OpenCV=True):
    # ROS Robotic reference frame (standard REP 103): X forward, Y left, Z up;
    # OpenCV reference frame: X right, Y down, Z up;
    # https://articulatedrobotics.xyz/tools/rotation-calculator/
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{tag_REP103}_to_{tag_OpenCV}',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '1.570796' if REP103_to_OpenCV else '-1.570796',
            '--frame-id', str(tag_REP103) if REP103_to_OpenCV else str(tag_OpenCV),
            '--child-frame-id', str(tag_OpenCV) if REP103_to_OpenCV else str(tag_REP103)
        ]
    )

    return static_tf

def generate_launch_description():
    pkg_handover_camera = get_package_share_directory('handover_camera')
    pkg_realsense = get_package_share_directory('realsense2_camera')
    
    tags_config_file = os.path.join(pkg_handover_camera, 'config', 'tags.yaml')

    # Static TF to set anchor tag to world
    static_tf_world_to_tag_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_tag_base',
        arguments=[
            '--x', '0.115',
            '--y', '-0.05',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'AprilTag_base_REP103'
        ]
    )

    # Static TF to convert tag_base from REP103 to OpenCV
    static_tf_tag_base_to_optical = get_static_tf_REP103_OpenCV(tag_REP103="AprilTag_base_REP103", tag_OpenCV="AprilTag_base_OpenCV", REP103_to_OpenCV=True)

    # # Static TF to convert detected AprilTags from OpenCV to REP103
    # static_tf_tags = []
    # with open(tags_config_file, 'r') as f:
    #     config = yaml.safe_load(f)

    # aprilTags = config.get('/**', {}).get('ros__parameters', {}).get('tag', {}).get('frames', [])

    # for aprilTag in aprilTags:
    #     ros_aprilTag = aprilTag.removeprefix("raw_") if aprilTag.startswith("raw_") else f"{aprilTag}_ros"
    #     static_tf_tags.append(get_static_tf_REP103_OpenCV(tag_REP103=ros_aprilTag, tag_OpenCV=aprilTag, REP103_to_OpenCV=False))


    # Driver Intel RealSense Depth Camera D435
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            # 'camera_name': 'realsense1',
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

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[
            tags_config_file,
            {'qos_overrides./camera/camera/color/camera_info.subscription.reliability': 'best_effort'},
            {'qos_overrides./camera/camera/color/image_raw.subscription.reliability': 'best_effort'},
        ],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info')
        ]
    )

    camera_tf_node = Node(
        package='handover_camera',
        executable='camera_tf_broadcaster',
        name='camera_tf_node',
        output='screen'
    )

    return LaunchDescription(
        [   
            static_tf_world_to_tag_base,
            static_tf_tag_base_to_optical,
            # List of all static TFs for detected AprilTags
            # *static_tf_tags,
            realsense_node,
            apriltag_node,
            camera_tf_node,
        ]
    )
