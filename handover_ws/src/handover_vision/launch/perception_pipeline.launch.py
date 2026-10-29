import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Limits for the 3D workspace used in the vision pipeline (world coordinates)
    common_params = {
        'xlim': '[0.2, 0.95]',
        'ylim': '[-0.5, 0.5]',
        'zlim': '[0.0, 0.5]',
        '/rgb_topic': '/color/image_raw',
        '/depth_topic': '/aligned_depth_to_color/image_raw',
        '/camera_info_topic': '/aligned_depth_to_color/camera_info'
    }

    # Static transform
    tf_world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', 
                   '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0', 
                   '--frame-id', 'world', '--child-frame-id', 'base_link']
    )

    tf_world_to_marker = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_marker_table',
        arguments=['--x', '0.525', '--y', '0.05', '--z', '-0.01', 
                   '--qx', '0.0', '--qy', '0.0', '--qz', '0.7071', '--qw', '0.7071', 
                   '--frame-id', 'world', '--child-frame-id', 'marker_table']
    )

    tf_marker_to_realsense1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='marker_table_to_realsense1_link',
        arguments=['--x', '0.5814001674435142', '--y', '0.21036899658717187', '--z', '0.4476264182356543', 
                   '--qx', '-0.25999745305201943', '--qy', '-0.040189657804866843', '--qz', '0.9494481697661579', '--qw', '-0.17133540350418913', 
                   '--frame-id', 'marker_table', '--child-frame-id', 'realsense1_link']
    )

    tf_marker_to_realsense2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='marker_table_to_realsense2_link',
        arguments=['--x', '-0.5821276592147536', '--y', '0.32182449667349167', '--z', '0.40575526647589477', 
                   '--qx', '-0.029424950334439717', '--qy', '0.22140127866664966', '--qz', '-0.08875756770534318', '--qw', '0.9706922958866809', 
                   '--frame-id', 'marker_table', '--child-frame-id', 'realsense2_link']
    )

    hand_detector1 = Node(
        package='handover_vision',
        executable='hand_detector',
        name='hand_detector1',
        output='screen',
        parameters=[common_params, {'camera_name': 'realsense1'}],
        arguments=['realsense1']
    )

    hand_detector2 = Node(
        package='handover_vision',
        executable='hand_detector',
        name='hand_detector2',
        output='screen',
        parameters=[common_params, {'camera_name': 'realsense2'}],
        arguments=['realsense2']
    )

    kalman_filter = Node(
        package='handover_vision',
        executable='kalman_filter_node',
        name='kalman_filter_node',
        output='screen'
    )

    object_detector1 = Node(
        package='handover_vision',
        executable='object_detector',
        name='object_detector1',
        output='screen',
        parameters=[common_params, {'camera_name': 'realsense1'}],
        arguments=['realsense1']
    )

    object_detector2 = Node(
        package='handover_vision',
        executable='object_detector',
        name='object_detector2',
        output='screen',
        parameters=[common_params, {'camera_name': 'realsense2'}],
        arguments=['realsense2']
    )

    pointcloud_merger = Node(
        package='handover_vision',
        executable='pointcloud_merger',
        name='pointcloud_merger',
        output='screen'
    )

    return LaunchDescription([
        tf_world_to_base,
        tf_world_to_marker,
        tf_marker_to_realsense1,
        tf_marker_to_realsense2,
        hand_detector1,
        hand_detector2,
        kalman_filter,
        object_detector1,
        object_detector2,
        pointcloud_merger
    ])