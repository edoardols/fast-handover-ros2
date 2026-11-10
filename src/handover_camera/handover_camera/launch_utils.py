import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node

def get_kinova_moveit_config(custom_xacro=None, use_fake_hardware="true", initial_pose="home"):
    pkg_share = get_package_share_directory('handover_camera')

    yaml_filename = f"initial_positions_{initial_pose.lower()}.yaml"
    initial_positions_file = os.path.join(pkg_share, "config", yaml_filename)

    mappings = {
        "use_fake_hardware": str(use_fake_hardware),
        "dof": "7",
        "robot_ip": "192.168.1.10",
        "gripper": "robotiq_2f_85",
        "use_internal_bus_gripper_comm": "true",
        "username": "admin",
        "password": "admin",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "initial_positions_file": initial_positions_file,
        'use_sim_time': "false",
    }

    builder = MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")

    if custom_xacro:
        builder.robot_description(file_path=custom_xacro, mappings=mappings)
    else:
        builder.robot_description(mappings=mappings)

    return builder.to_moveit_configs()

def get_static_tf_REP103_OpenCV(tag_REP103="tag_REP103", tag_OpenCV="tag_OpenCV", REP103_to_OpenCV=True):
    # ROS reference frame(standard REP 103: X forward, Y left, Z up;
    # OpenCV: X right, Y down, Z up:
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