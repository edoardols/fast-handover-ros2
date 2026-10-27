from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_no1_arg = DeclareLaunchArgument(
        'serial_no_cam1',
        default_value='213622076221',
        description='Serial number of the first RealSense (camera1)'
    )
    
    serial_no2_arg = DeclareLaunchArgument(
        'serial_no_cam2',
        default_value='213622072093',
        description='Serial number of the second RealSense (camera2)'
    )

    camera1_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera1',
        namespace='camera1',
        parameters=[{
            'serial_no': LaunchConfiguration('serial_no_cam1'),
            'enable_color': True,
            'enable_depth': True,
            'pointcloud.enable': True,
            'align_depth.enable': True
        }],
        output='screen'
    )

    camera2_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera2',
        namespace='camera2',
        parameters=[{
            'serial_no': LaunchConfiguration('serial_no_cam2'),
            'enable_color': True,
            'enable_depth': True,
            'pointcloud.enable': True,
            'align_depth.enable': True
        }],
        output='screen'
    )

    return LaunchDescription([
        serial_no1_arg,
        serial_no2_arg,
        camera1_node,
        camera2_node
    ])