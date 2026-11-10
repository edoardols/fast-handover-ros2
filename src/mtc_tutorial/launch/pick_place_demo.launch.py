from launch import LaunchDescription
from launch_ros.actions import Node

from mtc_tutorial.launch_utils import get_kinova_moveit_config

def generate_launch_description():
    moveit_config = get_kinova_moveit_config()

    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_task_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([pick_place_demo])