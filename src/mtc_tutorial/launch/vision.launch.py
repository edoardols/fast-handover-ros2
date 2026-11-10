import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from mtc_tutorial.launch_utils import get_static_tf_REP103_OpenCV

def generate_launch_description():
    pkg_share = get_package_share_directory('mtc_tutorial')
    pkg_realsense = get_package_share_directory('realsense2_camera')
    tags_config = os.path.join(pkg_share, 'config', 'tags.yaml')

    ### Static TF to set anchor tag to world
    anchor_tf = Node(
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
            '--child-frame-id', 'tag_base'
        ]
    )

    ### Static TF to convert tag_base from REP103 to OpenCV
    static_tf_tag_base_to_optical = get_static_tf_REP103_OpenCV(tag_REP103="tag_base", tag_OpenCV="tag_base_optical", REP103_to_OpenCV=True)

    ### Static TF to convert detected AprilTags from OpenCV to REP103
    static_tf_tags = []
    with open(tags_config, 'r') as f:
        config = yaml.safe_load(f)

    aprilTags = config.get('/**', {}).get('ros__parameters', {}).get('tag', {}).get('frames', [])

    for aprilTag in aprilTags:
        ros_aprilTag = aprilTag.removeprefix("raw_") if aprilTag.startswith("raw_") else f"{aprilTag}_ros"
        static_tf_tags.append(get_static_tf_REP103_OpenCV(tag_REP103=ros_aprilTag, tag_OpenCV=aprilTag, REP103_to_OpenCV=False))


    ### Driver Intel RealSense Depth Camera D435
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'rgb_camera.color_profile': '1280x720x30', # High precision
            # 'rgb_camera.color_profile': '640x480x15', # Low precision
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

    ### AprilTag ROS Node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        parameters=[
            tags_config,
            {'qos_overrides./camera/camera/color/camera_info.subscription.reliability': 'best_effort'},
            {'qos_overrides./camera/camera/color/image_raw.subscription.reliability': 'best_effort'},
        ],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info')
        ]
    )

    ### Camera TF Node
    camera_tf_node = Node(
        package='mtc_tutorial',
        executable='camera_tf_broadcaster',
        name='camera_tf_broadcaster',
        output='screen'
    )

    # Ritarda l'avvio di AprilTag e del Broadcaster di 5 secondi
    delayed_vision_nodes = TimerAction(
        period=2.0,
        actions=[
            apriltag_node,
            camera_tf_node,
        ]
    )

    return LaunchDescription(
        [
            realsense_launch,
            # apriltag_node,
            # camera_tf_node,
            anchor_tf,
            static_tf_tag_base_to_optical,
            *static_tf_tags,
            delayed_vision_nodes,
        ]
    )
