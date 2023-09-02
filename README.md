# HURON-Model
URDF model for HURON
## Introduction
This repository contains support packages for the HURON humanoid robot.

Included packages:
- `huron_description`: The URDF/xacro model of HURON.
- `huron_control`: The controllers of HURON.

Currently, 3 models of HURON are being developed and tested:
- `original-design`: The original robot foot design
- `flat-feet`: Experimental - adding a flat layer to the bottom of the foot to increase stability.
- `load-cell`: Currently the most stable - Using 2 6-DoF force-torque sensors at ankles to measure force

Branch naming convention:
- `main`: The most stable and best-performing version. Currently, main contains the `original-design`
- `-stable`: Stable branch.
- `-dev`: Development branch.
## How to run

**Pre-requisites:**
- Ubuntu 20.04, [ROS-Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
After installing ROS-Noetic run this command to automatically source ROS script every time you start a terminal:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
- Install gazebo-ros packages for gazebo-ros integration and ros-controllers for control interface:
```
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-ros-controllers
```
- Install git for cloning the HURON packages and contribute to the repo:
```
sudo apt update
sudo apt install git
```
**Setup and run HURON-Model**
- The packages need to be inside a catkin workspace. To create a catkin workspace:
```
mkdir -p <directory-name>/src
cd <directory-name>
catkin_make
```
- Then clone the HURON packages in the src folder using following commands:
```
cd src
git clone https://github.com/dtbpkmte/HURON-Model.git
```
- Then run the following commands to go to root directory of the workspace and build the workspace:
```
cd ..
catkin_make
```
- Then source the local workspace using following commands:
```
source devel/setup.bash
```
- Remember to source this local workspace every time you open a new terminal. To avoid doing that you can add this in the bash script of your terminal using following commands:
```
echo "source ~/<directory-name>/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

```
- Run following commands to start HURON simulation:
```
roslaunch huron_description gazebo.launch
```
- To switch branch:
```
git checkout <branch-name>
```
More information of using git can be found at https://git-scm.com/docs/gittutorial.
