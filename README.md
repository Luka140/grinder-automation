# grinder-automation

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
- [`rws_motion_client`](https://github.com/Luka140/rws_motion_client): Client node controlling ABB robot arm motion using the ABB robot driver: (`abb_ros2`).

CAUTION: `rws_motion_client` is a repository for moving grinder on a robot, and for grinder on a robot, the following repositories also need to be checked out in moving-grinder branch: 
`data_gathering`, `data_gathering_msgs`, `pcl_processing_ros2`, `grinder_model`, and `bag_converter`. For stationary grinding, exclude rws_motion_client from building.


# grinder-automation Dependencies
The `requirement.txt` and `setup.sh` contains all dependencies required for all the repositories outlined in the `.repos` file:

- `ros-humble-rosbag2-storage-mcap`: Enabled MCAP storage format for rosbags
- `open3d`: for pointcloud operations. Note that the used (and currently latest) version requires Numpy < 1.25. Used in `pcl_processing_ros2` and `lls_processing`.
- `libsdl2-dev`: Simple DirectMedia Layer(SDL) 2.0 for keyboard repository
- `keyboard_msgs`: keyboard ROS2 messages for UR trajectory recording. Used in `ur_trajectory_controller`. Installation instruction uses a fork created by Luka140
- [`pyads`](https://github.com/stlehmann/pyads): A Python wrapper for TwinCAT ADS library. Used in `data_gathering`
- [`concave_hull`](https://github.com/panin-anan/concave_hull): A Python library to calculate concave hulls. Used in `pcl_processing_ros2`
- [`pyransac3d`](https://github.com/leomariga/pyRANSAC-3D): A python library for the RANSAC algorithm. Used in`pcl_processing_ros2`
- `abb_ros2`: ABB robot ROS2 driver. Used in `rws_motion_client`

Aside from above dependencies, the following third party packages are also required to be installed as dependencies by using `setup_scancontrol.sh` on your docker.

- [`aravis 0.8.30`](https://github.com/AravisProject/aravis/releases/download/0.8.30/aravis-0.8.30.tar.xz)
- [`scancontrol Linux SDK 1.0.0`](https://software.micro-epsilon.com/scanCONTROL-Linux-SDK-1-0-0.zip)

see Installation section for installation instructions

## Installation

To install the dependencies
```
sudo apt update
sudo apt-get install ros-humble-ur
sudo apt-get install ros-humble-rosbag2-storage-mcap
sudo apt-get install libsdl2-dev

cd src
git clone https://github.com/Luka140/ros2-keyboard.git
git clone https://github.com/panin-anan/abb_ros2.git
cd ..

pip install pyads==3.4.2
pip install open3d==0.18.0
pip install numpy==1.24.0
pip install pyransac3d==0.6.0
```

To install aravis and scancontrol SDK with `setup_scancontrol.sh`:

```bash
$DEV_WORKSPACE/src/grinder-automation/setup_scancontrol.sh 
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

## Hardware setup
This grinder-automation repository works with the following hardware setup for an automated grind:

- PLC computer: CX8190 from [`Beckhoff Automation`](https://www.beckhoff.com/en-en/products/ipc/embedded-pcs/cx8100-arm-r-cortex-r-a9/cx8190.html)
- Universal Robot Arm: UR16e from [`Universal Robots`](https://www.universal-robots.com/products/ur16e/)
- Active Contact Flange: ACF-K from [`Ferrobotics`](https://www.ferrobotics.com/en/services/products/active-contact-flange-kit/)
- Laser Line Scanner: scanControl 29x0 from [`micro-epsilon`](https://www.micro-epsilon.com/2d-3d-measurement/laser-profile-scanners/scancontrol-29x0/?sLang=en) 
- ABB Robot Arm: (for moving grinder) ABB IRB-1200 from [`ABB Robotics`](https://new.abb.com/products/robotics/robots/articulated-robots/irb-1200)

PLC computer also contains extra IO Field Buses as necessary to connect to a [`flow control valve`](https://www.festo.com/us/en/a/8041715/), which controls the flow rate and RPM of the [`Dynafile`](https://www17.dynabrade.com/get-manual.php?c=PD01.73R) grinding power tool

## Test Run Example

First, source installed packages
```bash
source install/setup.bash
```

Before running the entire system, we can check each component by launching the following drivers one by one:

```bash
ros2 launch ur_trajectory_controller ur_driver.launch.py
ros2 launch ferrobotics_acf acf.launch.py
ros2 launch micro_epsilon_scancontrol_driver load_driver.launch.py
```

When all components are confirmed to be connectable, launch with data_gathering launch file with the following launch parameters:

```bash
ros2 launch data_gathering data_gathering.launch.py
```



