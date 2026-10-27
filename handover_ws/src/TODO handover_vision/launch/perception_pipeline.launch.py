import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('handover_vision')
    params_file = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    world_to_base_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_base_link',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '1.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'world',
                '--child-frame-id', 'base_link'
            ]
    )

    world_to_marker_table = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_marker_table',
        arguments=[
            '--x', '0.525', '--y', '0.05', '--z', '0.05',
            '--qx', '0.0', '--qy', '0.0', '--qz', '0.7071', '--qw', '0.7071',
            '--frame-id', 'world',
            '--child-frame-id', 'marker_table'
        ]
    )
    
    tf_marker_table_to_realsense1_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='marker_table_to_realsense1_link',
        arguments=[
            '--x', '0.5814001674435142', '--y', '0.21036899658717187', '--z', '0.4476264182356543',
            '--qx', '-0.25999745305201943', '--qy', '-0.040189657804866843', '--qz', '0.9494481697661579', '--qw', '-0.17133540350418913',
            '--frame-id', 'marker_table',
            '--child-frame-id', 'camera1_link'
        ]
    )

    tf_marker_table_to_realsense2_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='marker_table_to_realsense2_link',
        arguments=[
            '--x', '0.5821276592147536', '--y', '0.32182449667349167', '--z', '0.40575526647589477',
            '--qx', '-0.029424950334439717', '--qy', '0.22140127866664966', '--qz', '-0.08875756770534318', '--qw', '0.9706922958866809',
            '--frame-id', 'marker_table',
            '--child-frame-id', 'camera2_link'
        ]
    )

    hand_detector = Node(
        package='handover_vision',
        executable='hand_detector',
        name='hand_detector',
        output='screen'
    )

    object_detector = Node(
        package='handover_vision',
        executable='object_detector',
        name='object_detector',
        output='screen'
    )

    kalman_filter = Node(
        package='handover_vision',
        executable='kalman_filter_node',
        name='kalman_filter_node',
        output='screen'
    )

    pointcloud_merger = Node(
        package='handover_vision',
        executable='pointcloud_merger',
        name='pointcloud_merger',
        output='screen'
    )

    return LaunchDescription([
        world_to_base_link,
        world_to_marker_table,
        tf_marker_table_to_realsense1_link,
        tf_marker_table_to_realsense2_link,
        hand_detector,
        object_detector,
        kalman_filter,
        pointcloud_merger
    ])