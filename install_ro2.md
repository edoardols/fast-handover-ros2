### STEP 1: Installation ROS2 (Humble)
Ubuntu 22.04.5 LTS (Jammy Jellyfish): https://releases.ubuntu.com/jammy/
ROS 2 (Humble): https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

### STEP 2: Installation Kortex (Kinova driver)
Kinova Gen3 7 dof + driver ROS2 Kortex: https://github.com/Kinovarobotics/ros2_kortex/tree/humble

1. Open terminal
    - cd fast-handover-ros2

2. update system tools
    - sudo apt update
    - sudo apt install python3-colcon-common-extensions python3-vcstool

3. building from source
    - git clone -b humble --single-branch https://github.com/Kinovarobotics/ros2_kortex.git src/ros2_kortex
    - vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.humble.repos
    - vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.humble.repos

4. additional simulation packages
    - vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos

5. install dependences
    - rosdep install --from-paths src --ignore-src -y

6. build the workspace
    - colcon build --symlink-install

7. check installation
    - ros2 launch kortex_description view_robot.launch.py

8. MoveIt: https://moveit.ai/install-moveit2/binary/ e docs: https://moveit.picknik.ai/main/index.html
    - sudo apt install ros-humble-kinova-gen3-7dof-robotiq-2f-85-moveit-config
    - sudo apt install ros-humble-moveit

9. Cyclone DDS
    - sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

10. Test Virtual Hardware
    - ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true

11. Test Real-Life Hardware
    - ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py \
  robot_ip:=192.168.1.10

### STEP 3: Requirements
To build the workspace
1. Open terminal
    - cd fast-handover-ros2

2. Build the workspace
    - colcon build

3. If the required dependancy are not installed
    - rosdep install --from-paths src --ignore-src -y

# Start a simulation (or real robot)
0. Edit launch_kernel.sh file with the configuration desired
1. Open terminal
    - cd fast-handover-ros2
2. Edit launch_kernel.sh if needed
    - chmod +x launch_kernel.sh
3. Run the bash
    - ./launch_kernel.sh
