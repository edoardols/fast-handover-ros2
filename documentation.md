source /opt/ros/humble/setup.bash
source install/setup.bash

### Handover Camera
- /config/tags.yaml
- /launch/camera.launch.py
- /src/camera_tf_broadcaster.cpp

Has the following topics:
/camera/{camera_name}/color/camera_info
/camera/{camera_name}/color/image_raw
/camera/{camera_name}/color/image_raw/compressed
/camera/{camera_name}/color/image_raw/compressedDepth
/camera/{camera_name}/color/image_raw/theora
/camera/{camera_name}/color/metadata
/camera/{camera_name}/extrinsics/depth_to_color

/clicked_point
/detections
/goal_pose

/initialpose
/parameter_events
/rosout
/tf
/tf_static



### Handover Vision
python3 -m venv venv_vision --system-site-packages
touch venv_vision/COLCON_IGNORE
source venv_vision/bin/activate
source /opt/ros/humble/setup.bash

sudo apt install ros-humble-tf2-sensor-msgs ros-humble-vision-msgs


- /handover_vision/hand_detector.py