import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_servo_ctrl = get_package_share_directory('servo_ctrl')

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='panda',
        description='Robot model: panda or ur5'
    )

    config_panda = os.path.join(pkg_servo_ctrl, 'config', 'config_panda.yaml')
    config_ur5 = os.path.join(pkg_servo_ctrl, 'config', 'config_ur5.yaml')

    spacenav_node = Node(
        package='spacenav',
        executable='spacenav_node',
        name='spacenav_node',
        output='screen'
    )

    spacenav_to_twist_node = Node(
        package='moveit_servo',
        executable='spacenav_to_twist',
        name='spacenav_to_twist',
        output='screen'
    )

    servo_node_panda = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_server',
        output='screen',
        parameters=[
            config_panda, 
            {'parameter_ns': 'optional_parameter_namespace'}
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('robot'), "' == 'panda'"])
        )
    )

    servo_node_ur5 = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_server',
        output='screen',
        parameters=[
            config_ur5,
            {'parameter_ns': 'optional_parameter_namespace'}
        ],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('robot'), "' == 'ur5'"])
        )
    )

    return LaunchDescription([
        robot_arg,
        spacenav_node,
        spacenav_to_twist_node,
        servo_node_panda,
        servo_node_ur5
    ])