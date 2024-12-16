# grinder-automation

Todo add third party pkgs installed through apt, like abb driver, ur driver.

## Overview
This repository contains instructions and necesssary files for setting up a workspace to perform automated testing of material removal using a pneumatic grinder. 

# grinder-automation Repositories
The `.repos` file contain references to all repositories required to operate either stationary grinding tests or robot-arm mounted grinding tests.
The following are the repositories that will be installed using `.repos`: 
- [`data_gathering`](https://github.com/Luka140/data_gathering): Contain main launch file for the test, and control the grinder
- [`stamped_std_msgs`](https://github.com/Luka140/stamped_std_msgs/tree/main): Stamped standard messages for data storage
- [`data_gathering_msgs`](https://github.com/Luka140/data_gathering_msgs): Additional msgs
- [`ur_trajectory_controller`](https://github.com/Luka140/ur_trajectory_controller): Controls the UR16e
- [`scancontrol`](https://github.com/Luka140/scancontrol/tree/ros2-devel): ROS2 Driver for Scancontrol laser line scanners
- [`lls_processing`](https://github.com/Luka140/lls_processing): Compiles 3D pointcloud reconstructions based on TF transfers
- [`pcl_processing_ros2`](https://github.com/panin-anan/pcl_processing_ros2/tree/main): Used to calculate volume loss between two scans
- [`Universal_Robots_ROS2_Driver`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble): Driver for the UR16e
- [`ferrobotics_acf`](https://github.com/Luka140/ferrobotics_acf/tree/humble): Controls the ACF
- [`grinder_model`](https://github.com/panin-anan/grinder_model): Python script for training and evaluating predictive model with test data
- [`bag_converter`](https://github.com/Luka140/bag_converter): Python script for postprocessing test data from rosbag into .csv data
- [`rws_motion_client`](https://github.com/Luka140/rws_motion_client): Driver for the ABB robot.

CAUTION: `rws_motion_client` is a repository for moving grinder on a robot, and for grinder on a robot, the following repositories also need to be checked out in moving-grinder branch: 
`data_gathering`, `data_gathering_msgs`, `pcl_processing_ros2`, `grinder_model`, and `bag_converter`. For stationary grinding, exclude rws_motion_client from building.


# grinder-automation Dependencies
The `requirement.txt` lists all dependencies required for all the repositories outlined in the `.repos` file:

- `ros-humble-rosbag2-storage-mcap`: Enabled MCAP storage format for rosbags
- `open3d`: for pointcloud operations. Note that the used (and currently latest) version requires Numpy < 1.25. Used in `pcl_processing_ros2` and `lls_processing`.
- [`pyads`](https://github.com/stlehmann/pyads): A Python wrapper for TwinCAT ADS library. Used in `data_gathering`
- [`concave_hull`](https://github.com/panin-anan/concave_hull): A Python library to calculate concave hulls. Used in `pcl_processing_ros2`
- [`pyransac3d`](https://github.com/leomariga/pyRANSAC-3D): A python library for the RANSAC algorithm. Used in`pcl_processing_ros2`

see Installation section for installation instructions

## Installation

To install the dependencies
```
sudo apt-get install ros-humble-ur
sudo apt-get install ros-humble-rosbag2-storage-mcap

pip install pyads==3.4.2
pip install open3d==0.18.0
pip install numpy==1.24.0
pip install pyransac3d==0.6.0
```

To install the repositories, first, update your version control system tool
```bash
sudo apt update
sudo apt install python3-vcstool
```

Then, navigate to your workspace source folder (e.g., ~/workspace_folder/src)
```bash
cd src
```

clone this repositories to your workspace source folder
and run vcs import using the path to the `.repos` file, for example the author workspace name is 'BrightSkyRepoLinux':
```bash
git clone https://github.com/Luka140/grinder-automation.git
vcs import < /workspaces/BrightSkyRepoLinux/src/grinder-automation/.repos
```

NOTE: For robot-arm mounted grinding test setup, check out moving-grinder branch of this repository.


To build all the packages, follow below sequence because some repositories are dependent on the ROS2 message type repositories

```bash
colcon build --packages-select stamped_std_msgs
colcon build --symlink-install
```


For more information on the usage, Check out https://github.com/Luka140/data_gathering
