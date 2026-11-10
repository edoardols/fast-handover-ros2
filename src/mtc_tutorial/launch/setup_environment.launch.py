from launch import LaunchDescription
from launch_ros.actions import Node

from mtc_tutorial.launch_utils import get_kinova_moveit_config

def generate_launch_description():
    moveit_config = get_kinova_moveit_config()

    setup_environment_node = Node(
        package="mtc_tutorial",
        executable="setup_environment_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([setup_environment_node])