FROM ros:noetic-ros-core

# Set environment variables for ROS
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WORKSPACE=/workspace/handover_ws

# FIX GPG ROS 
RUN set -eux; \
    # 1) remove existing repo ROS
    rm -f /etc/apt/sources.list.d/ros*.list /etc/apt/sources.list.d/ros2*.list || true; \
    apt-get update; \
    apt-get install -y --no-install-recommends curl gnupg ca-certificates; \
    mkdir -p /etc/apt/keyrings; \
    # 2) keyring ROS
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg; \
    # 3) repo ROS + signed-by
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros1-latest.list; \
    # 4) update
    apt-get update

# Install all system and ROS dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libpoco-dev \
    libeigen3-dev \
    libssl-dev \
    libcurl4-openssl-dev \
    libbenchmark-dev \
    dpkg-dev \
    fakeroot \
    libyaml-cpp-dev \
    python3-pip \
    python3-catkin-tools \
    python3-vcstool \
    ros-noetic-combined-robot-hw \
    ros-noetic-controller-interface \
    ros-noetic-controller-manager \
    ros-noetic-gazebo-dev \
    ros-noetic-hardware-interface \
    ros-noetic-industrial-robot-status-interface \
    ros-noetic-joint-limits-interface \
    ros-noetic-moveit-commander \
    ros-noetic-moveit-planners-ompl \
    ros-noetic-moveit-ros-visualization \
    ros-noetic-moveit-ros-move-group \
    ros-noetic-moveit-servo \
    ros-noetic-moveit-simple-controller-manager \
    ros-noetic-ompl* \
    ros-noetic-pass-through-controllers \
    ros-noetic-pilz-industrial-motion-planner \
    ros-noetic-realtime-tools \
    ros-noetic-rviz \
    ros-noetic-scaled-joint-trajectory-controller \
    ros-noetic-smach \
    ros-noetic-smach-msgs \
    ros-noetic-smach-ros \
    ros-noetic-soem \
    ros-noetic-speed-scaling-interface \
    ros-noetic-speed-scaling-state-controller \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-tf2 \
    ros-noetic-tf2-eigen \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-kdl \
    ros-noetic-tf2-msgs \
    ros-noetic-tf2-py \
    ros-noetic-tf2-ros \
    ros-noetic-trac-ik-kinematics-plugin \
    ros-noetic-transmission-interface \
    ros-noetic-ur-client-library \
    ros-noetic-ur-robot-driver \
    ros-noetic-ur-calibration \
    ros-noetic-ur-msgs \
    ros-noetic-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install pymodbus==2.5.3 pyserial-asyncio

# Build and install libfranka 0.9.2
WORKDIR /
RUN git clone --branch 0.9.2 --depth 1 https://github.com/frankaemika/libfranka.git && \
    cd libfranka && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. && \
    make -j$(nproc) && \
    cpack -G DEB && \
    dpkg -i libfranka*.deb

# Create catkin workspace
RUN mkdir -p ${ROS_WORKSPACE}/src
WORKDIR ${ROS_WORKSPACE}/src

# Clone external dependencies
RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git && \
    git clone --branch 1.3.3 https://github.com/ros-industrial/universal_robot.git && \
    git clone https://github.com/jr-robotics/robotiq.git && \
    rm -rf robotiq/robotiq_3f* robotiq/robotiq_2f_140_gripper_visualization && \
    git clone https://github.com/a-price/robotiq_arg85_description.git && \
    git clone --branch 0.9.1 https://github.com/frankaemika/franka_ros.git && \
    rm -rf franka_ros/franka_gazebo && \
    git clone --branch noetic-devel https://github.com/moveit/panda_moveit_config.git

# Copy local packages into the workspace
COPY ./ ${ROS_WORKSPACE}/src/

# Make scripts executable
RUN chmod +x /workspace/handover_ws/src/smach_handover_ctrl/scripts/*

# Build the ROS workspace
WORKDIR ${ROS_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin init && catkin build"

# Source the workspace on container startup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source ${ROS_WORKSPACE}/devel/setup.bash" >> ~/.bashrc

# Set ROS network environment variables (Edit the network configuration for your convenience)
RUN echo "export ROS_MASTER_URI=http://172.31.1.1:11311" >> ~/.bashrc && \
    echo "export ROS_IP=172.31.1.2" >> ~/.bashrc
