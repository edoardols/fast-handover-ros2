from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='ur5',
        description='Robot model: ur5 or panda'
    )

    robot_param = {'robot': LaunchConfiguration('robot')}

    controller_server_node = Node(
        package='smach_handover_ctrl',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[robot_param]
    )

    planner_node = Node(
        package='smach_handover_ctrl',
        executable='planner',
        name='planner',
        output='screen',
        parameters=[robot_param]
    )

    return LaunchDescription([
        robot_arg,
        controller_server_node,
        planner_node
    ])