# Vision pipeline

This repository provides guidelines and necessary scripts to run the vision-based pipeline for human-to-robot handover. The setup is tested with ROS Noetic on Ubuntu 20.04 using two Intel Realsense D435i cameras.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Docker Setup](#docker-setup)
3. [System Calibration](#system-calibration)
4. [Launching the Pipeline](#launching-the-pipeline)
5. [Visualization](#visualization)

---

## Prerequisites


Ensure you have the following:

- Ubuntu 20.04 with ROS Noetic installed
- 2 x Intel® RealSense™ D400 series
- NVIDIA GPU with CUDA support recommended for faster inference
- [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) and [NVIDIA Docker Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) installed

---

## System Calibration

- Calibrate the cameras and update the transformations `world_to_marker_table` `marker_table_to_realsense1_link` `marker_table_to_realsense2_link` in `perception_pipeline.launch`.

- Customize the workspace limits (`xlim`, `ylim`, `zlim`) in the same file.

- Update the serial numbers `serial_no` of the cameras in the `camera.launch` file 

---
## Docker Setup

### Build the Docker Image

Create a Docker image with the necessary dependencies:

```bash
docker build -t ros-handover:vision .
```

### Run the Docker Container

Start the Docker container with GPU support:

```bash
docker run --runtime=nvidia --gpus all --net=host \
  --privileged \
  --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
  --device /dev/nvidia0:/dev/nvidia0 \
  --device /dev/nvidiactl:/dev/nvidiactl \
  --device /dev/nvidia-uvm:/dev/nvidia-uvm \
  --device /dev/nvidia-uvm-tools:/dev/nvidia-uvm-tools \
  --device /dev/usb \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -it ros-handover:vision
```


> 💡 Tip: To enable live code editing or to add updated files (e.g., calibration data) without rebuilding the image, you can mount a local folder using the -v flag, and then copy files from there:
>
> ```bash
> -v /path/on/host:/path/in/container
> ```
>
> For example, if the `handover_vision` package is in `workspace` folder in your host, you can run:
>
> ```bash
> -v ~/workspace:/workspace/shared
> ```
>
> Once inside the container, you can copy or use the files as needed:
>
> ```bash
> cp -r /workspace/shared/handover_vision workspace/handover_ws/src/
> ```
---
## Launching the Pipeline

Before starting, ensure `roscore` is running on the ROS master machine.

1. Inside the running Docker container, start the cameras: 

```bash
roslaunch handover_vision camera.launch
```
2. Launch the perception pipeline, by opening a new terminal on your host and attaching to the running container:
    ```bash
    docker exec -it <container_name> bash
    ```
    Then run:
    ```bash
    roslaunch handover_vision perception_pipeline.launch
    ```
    ℹ️ On first run, the required YOLO model weights will be automatically downloaded.

---
## Visualization

You can use `rviz` to visualize the perception pipeline (make sure to allow X11 forwarding if inside docker container).

To launch RViz with the pre-configured layout:

```bash
rosrun rviz rviz -d rviz/perception_pipeline.rviz

```

---




