FROM ros:noetic-ros-core

# Set environment variables for ROS
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WORKSPACE=/workspace/handover_ws

# Update and install system dependencies
RUN apt-get update && \
    apt-get install -y python3-pip && \
    apt-get install -y \
    ros-noetic-vision-msgs \
    ros-noetic-franka-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-tf \
    ros-noetic-realsense2-camera \
    ros-noetic-rgbd-launch \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-rviz


# Create workspace and copy the handover_vision content from the current directory
WORKDIR /workspace
RUN mkdir -p handover_ws/src
COPY ./ handover_ws/src/handover_vision/

# Make scripts executable
RUN chmod +x /workspace/handover_ws/src/handover_vision/scripts/*

# Install Python dependencies
WORKDIR /workspace/handover_ws/src/handover_vision/
RUN pip3 install --upgrade pip && \
    pip3 install -r requirements.txt


# Build the ROS workspace
WORKDIR ${ROS_WORKSPACE}
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source the workspace on container startup
RUN echo "source ${ROS_WORKSPACE}/devel/setup.bash" >> ~/.bashrc

RUN echo "export ROS_MASTER_URI=http://172.31.1.1:11311" >> ~/.bashrc && \
    echo "export ROS_IP=172.31.1.1" >> ~/.bashrc
