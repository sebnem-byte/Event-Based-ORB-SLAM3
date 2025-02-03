# Event-based ORB-SLAM3 ROS Node

## Overview
This project is a ROS1-based implementation of an event-based SLAM system that integrates **rpg_dvs_ros** and **EORB_SLAM** to process event camera data. The node operates on **Ubuntu 20.04 with ROS Noetic** and is designed to handle event-based datasets, particularly **ETHZ Public Event Dataset**. It utilizes ORB-SLAM3 for state estimation and event tracking.

## References
This project integrates components from:

- **[EORB_SLAM](https://github.com/m-dayani/EORB_SLAM)**: An event-based extension of ORB-SLAM3, developed by Masoud Dayani Najafabadi.

## Features
- **Event-based SLAM** with ORB-SLAM3 integration.
- **Time-synchronized event processing** with batch thresholding.
- **Event timestamp normalization and sorting** for improved accuracy.
- **Adaptive event batching** to optimize SLAM input.
- **Event-based mapping and tracking** for 6-DOF camera pose estimation.
- **Multi-threaded implementation** to ensure real-time performance.

## Dependencies
Make sure you have the following dependencies installed:

- **ROS Noetic** (Ubuntu 20.04)
- **rpg_dvs_ros** (for event camera integration)
- **EORB_SLAM** (event-based ORB-SLAM3 extension)
- **OpenCV 3.4.1**
- **PCL (Point Cloud Library)**
- **cv_bridge, image_transport, message_filters** (ROS message handling)

## Installation
1. Clone the required repositories:
   ```sh
   cd ~/catkin_ws/src
   git clone https://github.com/uzh-rpg/rpg_dvs_ros.git
   git clone https://github.com/m-dayani/EORB_SLAM.git
   ```
2. Install dependencies:
   ```sh
   sudo apt update && sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-message-filters libpcl-dev
   ```
3. Build the workspace:
   ```sh
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Usage
Run the **EORB-SLAM** node with an event dataset:
```sh
roslaunch eorb_slam3 eorbslam.launch settings:=/path/to/settings.yaml
```

## Event Processing Pipeline
1. **Event Data Handling:**
   - Events are collected in **vEvBuff**.
   - If event timestamps are too close, they are accumulated before being sent to SLAM.
   - The batch threshold ensures event accumulation before processing.

2. **SLAM Initialization:**
   - Loads **ORB Vocabulary** from EORB_SLAM.
   - Reads dataset configurations from **EvEthzLoader**.

3. **Event SLAM Execution:**
   - **ORB-SLAM3::System** processes the incoming event stream.
   - Tracks event features, builds a map, and estimates the camera pose.
   - Outputs **tracking state** and **map point statistics**.

4. **Data Logging:**
   - Saves trajectory and SLAM-generated map:
   ```sh
   /home/user/trajectory.txt
   /home/user/orb_map.txt
   ```

## Configuration File: `EvETHZ.yaml`
Key parameters configured in `EvETHZ.yaml`:

- **Dataset Path:**
  ```yaml
  DS.name: "ev_ethz"
  Path.DS.root: "/ds_path/event/ethz"
  ```
- **Camera Calibration:**
  ```yaml
  Camera.fx: 199.092
  Camera.fy: 198.828
  Camera.cx: 132.192
  Camera.cy: 110.712
  ```
- **Event Windowing Parameters:**
  ```yaml
  Event.data.l1ChunkSize: 2000
  Event.data.minEvGenRate: 1.0
  Event.data.maxPixelDisp: 3.0
  ```

## Acknowledgments
This project integrates components from:
- **[rpg_dvs_ros](https://github.com/uzh-rpg/rpg_dvs_ros)** for event camera processing.
- **[EORB_SLAM](https://github.com/m-dayani/EORB_SLAM)** for event-based SLAM implementation.

# Troubleshooting Installation Issues for ORB-SLAM3 with ROS

## Overview

This guide helps in solving common **installation issues** while setting up **ORB-SLAM3** with **ROS** for event-based SLAM using Dynamic Vision Sensors (DVS). If you're encountering issues related to missing dependencies, incorrect configurations, or build problems, this guide will walk you through troubleshooting and resolving them.

## Pre-requisites

Before starting, ensure you have the following installed:

- **ROS Noetic** (or compatible ROS version)
- **ORB-SLAM3** installed on your system
- **DVS/Camera Drivers** (if using DAVIS or event cameras)
- **OpenCV** (version 4.x, compatible with ROS Noetic)
- **Eigen**, **CMake**, and other dependencies
- **Catkin tools** for building ROS packages

---
# Troubleshooting Installation Issues for ORB-SLAM3 with ROS

## Overview

This guide provides solutions to common **installation issues** encountered while setting up **ORB-SLAM3** with **ROS** for event-based SLAM using Dynamic Vision Sensors (DVS). If you're experiencing problems related to missing dependencies, incorrect configurations, or build errors, follow this guide for troubleshooting.

## Pre-requisites

Before starting, ensure that you have the following installed:

- **ROS Noetic** (or a compatible ROS version)
- **ORB-SLAM3** installed in your system
- **DVS/Camera Drivers** (if using DAVIS or other event cameras)
- **OpenCV** (version 4.x, compatible with ROS Noetic)
- **Eigen**, **CMake**, and other dependencies
- **Catkin tools** for building ROS packages

## Common Installation Issues & Solutions

### Issue 1: Missing ROS Package `orb_slam3`

#### Error:
`rosrun orb_slam3 orbslam_node`  
`[ERROR] [ros]: Could not find package orb_slam3`

#### Fix:  
Ensure that `ORB_SLAM3` is cloned into `catkin_ws/src` and properly built:  

`cd ~/catkin_ws/src`  
`git clone https://github.com/UZH-RPG/ORB_SLAM3.git`  
`cd ~/catkin_ws`  
`catkin_make`  
`source devel/setup.bash`  

### Issue 2: Missing OpenCV Header Files or Linking Errors

#### Error:
`fatal error: opencv2/core/core.hpp: No such file or directory`

#### Fix:  
Ensure OpenCV is installed and linked correctly:  

`sudo apt-get update`  
`sudo apt-get install libopencv-dev`  

### Issue 3: Missing `catkin_make` Dependencies

#### Error:
`CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:125 (find_package): Could not find a package configuration file provided by "ORB_SLAM3"`

#### Fix:  
Make sure `ORB_SLAM3` is inside `catkin_ws/src`, then run:  

`source ~/catkin_ws/devel/setup.bash`  
`catkin_make`  

### Issue 4: CMake Errors with `Eigen` Library

#### Error:
`CMake Error at CMakeLists.txt:13 (find_package): Could not find a package configuration file provided by "Eigen3"`

#### Fix:  
Install the missing Eigen3 library:  

`sudo apt-get install libeigen3-dev`  

If the error persists, manually link Eigen3 in your `CMakeLists.txt`.  

### Issue 5: Camera or IMU Driver Errors

#### Error:
`[ERROR] Cannot load settings file.`  

#### Fix:  
- Verify the **event camera driver** is correctly installed.  
- Ensure the dataset path is correctly set in the YAML file.  
- Check that the camera topics are correctly published (`/dvs/image_raw`, `/dvs/events`, `/dvs/imu`).  

### Issue 6: Missing ROS Dependencies (`dvs_msgs`, `sensor_msgs`, etc.)

#### Error:
`[rosrun] Unable to locate package dvs_msgs`

#### Fix:  
Install missing ROS message packages:  

`sudo apt-get install ros-noetic-dvs-msgs`  
`sudo apt-get install ros-noetic-sensor-msgs`  

Then, rebuild your workspace:  

`cd ~/catkin_ws`  
`catkin_make`  
`source devel/setup.bash`  

### Issue 7: CMake Version Compatibility

#### Error:
`CMake Error at CMakeLists.txt:16 (message): CMake version >= 3.10 is required.`

#### Fix:  
Ensure you have the correct version of CMake installed:  

`sudo apt-get install cmake`  

If a newer version is needed:  

`sudo apt-get remove cmake`  
`sudo snap install cmake --classic`  

### Issue 8: Permission Errors (when accessing dataset files)

#### Error:
`[ERROR] Failed to access dataset path.`  

#### Fix:  
Ensure the dataset files have the correct permissions:  

`chmod -R 755 /path/to/your/dataset`  

## Conclusion

This guide covers the most common issues when setting up ORB-SLAM3 for event-based SLAM in ROS. Ensure all dependencies are installed and correctly linked, and always source your workspace:  

`source ~/catkin_ws/devel/setup.bash`  

If you still encounter issues, consider checking ROS logs:  

`roslaunch package_name node_name.launch --screen`  

Feel free to report any additional problems!


## License
This project is open-source and follows the licensing terms of EORB_SLAM and rpg_dvs_ros.

