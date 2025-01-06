# TrayTracker_ROS2
A ros2 system for tracking and counting objects processed as arrays.

![TrayTracker_ROS2_GIF.gif](TrayTracker_ROS2_GIF.gif)

## Overview
![rosgraph.png](rosgraph.png)

## Installation
### Global installation
Make sure the following is installed:
- [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
```bash
$ lsb_release -a
No LSB modules are available.
Distributor ID:	Ubuntu
Description:	Ubuntu 24.04.1 LTS
Release:	24.04
Codename:	noble

$ printenv | grep ROS_DISTRO
ROS_DISTRO=jazzy

$ python3 --version
Python 3.12.3

$ git clone https://github.com/AbdielAlfonsoRinconCantu/TrayTracker_ROS2.git
$ cd ~/TrayTracker_ROS2

$ sudo apt update
$ sudo apt install -y $(cat packages.list)

$ pip install -r requirements.txt 
```

### Building packages
**ROS2_Ab_C++**
```bash
$ cd ~/TrayTracker_ROS2/ROS2_Ab_C++/ros2_ws
$ source /opt/ros/jazzy/setup.bash
$ colcon build --cmake-clean-cache
```
<br>

**ROS2_Ab_OPCUA**
```bash
$ cd ~/TrayTracker_ROS2/ROS2_Ab_OPCUA
$ python3.12 -m venv ROS2_Ab_OPCUA_venv
$ source ~/ROS2_Ab_OPCUA/ROS2_Ab_OPCUA_venv/bin/activate
$ pip install -r ROS2_Ab_OPCUA_venv_requirements.txt
```

On a new terminal:
```bash
$ cd ~/TrayTracker_ROS2/ROS2_Ab_OPCUA/ros2_ws
$ source /opt/ros/jazzy/setup.bash
$ source ~/ROS2_Ab_OPCUA/ROS2_Ab_OPCUA_venv/bin/activate
$ colcon build --cmake-clean-cache
```
<br>

**ROS2_Ab_YOLO**
```bash
$ cd ~/TrayTracker_ROS2/ROS2_Ab_YOLO
$ python3.12 -m venv ROS2_Ab_YOLO_venv
$ source ~/ROS2_Ab_YOLO/ROS2_Ab_YOLO_venv/bin/activate
$ pip install -r ROS2_Ab_YOLO_requirements.txt
```

On a new terminal:
```bash
$ cd ~/TrayTracker_ROS2/ROS2_Ab_YOLO/ros2_ws
$ source /opt/ros/jazzy/setup.bash
$ source ~/ROS2_Ab_YOLO/ROS2_Ab_YOLO_venv/bin/activate
$ colcon build --cmake-clean-cache
```
<br>

**ROS2_Sarath18_OpenCV**
```bash
$ cd ~/TrayTracker_ROS2/ROS2_Sarath18_OpenCV/ros2_ws
$ source /opt/ros/jazzy/setup.bash
$ colcon build --cmake-clean-cache
```
<br>

**ROS2_marcos-moura97_Flask**
```bash
$ cd ~/TrayTracker_ROS2/ROS2_marcos-moura97_Flask/ros2_ws
$ source /opt/ros/jazzy/setup.bash
$ colcon build --cmake-clean-cache
```
<br>

**ROS2_marcos-moura97_Flask**
```bash
$ cd ~/TrayTracker_ROS2/ROS2_xovobobo_RTSP/ros2_ws
$ source /opt/ros/jazzy/setup.bash
$ colcon build --cmake-clean-cache
```

## Usage

## Acknowledgments

- Sarath18 / video.py: https://gist.github.com/Sarath18/7c48b6f2e667bf6dab1b12f419cab397
- marcos-moura97 / video_stream_ros2 : https://github.com/marcos-moura97/video_stream_ros2
- xovobobo / image2rtsp : https://github.com/xovobobo/image2rtsp
