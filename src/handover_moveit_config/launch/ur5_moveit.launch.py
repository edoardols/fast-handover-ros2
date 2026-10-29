#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    robot_ip_arg = DeclareLaunchArgument("robot_ip", default_value="172.32.1.87", description="IP of the UR5 robot")
    pipeline_arg = DeclareLaunchArgument("pipeline", default_value="ompl")

    db_arg = DeclareLaunchArgument("db", default_value="false")
    db_path_arg = DeclareLaunchArgument("db_path", default_value="default_warehouse_mongo_db")

    debug_arg = DeclareLaunchArgument("debug", default_value="false")

    load_robot_description_arg = DeclareLaunchArgument("load_robot_description", default_value="true")

    use_gui_arg = DeclareLaunchArgument("use_gui", default_value="false")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")

    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    run_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # [LEGACY ROS 1 MoveIt 2 Warehouse support requires external setup]
    # <include file="$(find ur5_moveit_config)/launch/default_warehouse_db.launch" if="$(arg db)">
    #   <arg name="moveit_warehouse_database_path" value="$(arg db_path)"/>
    # </include>

    return LaunchDescription([
        robot_ip_arg,
        pipeline_arg,
        db_arg,
        db_path_arg,
        debug_arg,
        load_robot_description_arg,
        use_gui_arg,
        use_rviz_arg,
        run_move_group_node,
        run_rviz_node
    ])