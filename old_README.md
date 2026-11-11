# Fast Human-Robot Handover ROS2

This repository provides the packages to perform vision-based human-to-robot fast handover.
The code is tested on **Ubuntu 20.04** with **ROS Noetic** and is hereby released for reproducibility. 

The system was developed for the Robotic Grasping and Manipulation Competition (RGMC) - Sub-Track on Human-to-Robot Handovers and won two consecutive editions ([2024 RGMC](https://corsmal.eecs.qmul.ac.uk/rgm24icra/), [2025 RGMC](https://corsmal.github.io/rgmc2025-handover-track/)).  For more details, refer to the [paper](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=11460206). 
 
### RGMC ICRA 24
[![RGMC ICRA 24 - Handover](https://img.youtube.com/vi/4wzR3eVm9P0/0.jpg)](https://www.youtube.com/watch?v=4wzR3eVm9P0)

### RGMC ICRA 25
[![RGMC ICRA 25 - Handover](https://img.youtube.com/vi/FGFlR_2N6G4/0.jpg)](https://www.youtube.com/watch?v=FGFlR_2N6G4)

---
### Reference

Please acknowledge the authors in any academic publication that uses parts of these codes.

```
@article{turco2026fast,
  title={Fast Human-Robot Handover using a Vision-Based Pipeline}, 
  author={Turco, Enrico and Castellani, Chiara and Salvietti, Gionata and Pozzi, Maria},
  journal={IEEE Robotics and Automation Practice}, 
  volume={1},
  number={},
  pages={66-70},
  year={2026},
  publisher={IEEE},
  doi={10.1109/RAP.2026.3680049}}
```

---

Our setup utilizes two PCs connected to the same local network, one for the **vision pipeline**, and the other for **robot control**. For the setup of the PC running the vision pipeline, please refer to the relative [guide](https://github.com/SIRSIIT/fast-handover/blob/main/handover_vision/README.md), while for the robot control, please refer to [this one](https://github.com/SIRSIIT/fast-handover/blob/main/handover_robot/README.md). 

## Network Configuration

This system assumes the following network layout:

- The two PCs are connected to the `172.31.1.*` subnet.
- The robot is on a separate network: `172.32.1.*`.

> ⚙️ You can optionally adjust these settings if needed.


### Vision PC (ROS Master):
- **IP Address:** `172.31.1.1`  
- **Netmask:** `255.255.255.0`

### Robot Control PC:
- **IP Address:** `172.31.1.2`  
- **Netmask:** `255.255.255.0`

In this setup, the **vision PC** serves as the **ROS master**, running `roscore`.

---

### ROS Environment Setup

If you change any IP addresses, make sure to update your `~/.bashrc` file on both machines and inside Docker containers:

```bash
export ROS_MASTER_URI=http://172.31.1.1:11311/
export ROS_IP=172.31.1.1
```

Then, apply the changes:

```bash
source ~/.bashrc
```
