
# tilting_drone_x4

This repository contains a simulation model of a tilting arm quadcopter and a controller to control the drone. The model is based on PX4-Autopilot SITL.

## Minimum Requirements
- ROS Humble
- Gazebo Garden
- Ubuntu 22.04

## ROS2 & GAZEBO Garden Installation
### To install ROS 2 "Humble" on Ubuntu 22.04:
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

```
Or you can refer this page:
[ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).


### To Install GAZEBO garden:
First install some necessary tools:

```bash
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
```

Then install Gazebo Garden:
```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden

```
Or you can refer this page:
[GAZEBO garden](https://gazebosim.org/docs/garden/install_ubuntu).


    

## Setup Instructions

### Clone the Repository
To clone the repository, run the following commands in your terminal:

```bash
mkdir -p workspace/drone_x4_ws/src
cd drone_x4_ws/src
git clone https://github.com/MayurRabadiya/tilting_drone_x4.git

```

### Setup Dependencies
```bash
cd tilting_drone_x4
sh setup_dependencies.sh

```
This process will take some time. Once the setup is complete, the workspace structure will look like this:

    drone_x4_ws
    ├── src
    │   ├── tilting_drone_x4
    │   └── tilting_drone_x4
    ├── PX4-Autopilot
    └── Micro-XRCE-DDS-Agent

## Running the Simulation
### Build the Workspace
```bash
cd drone_x4_ws
colcon build --symlink-install
source install/local_setup.bash

```

### Launch the Simulation
To run the tilting_drone_x4 simulation, execute the following command:
```bash
ros2 launch tilting_drone_x4 drone.launch.py
```

If you want to run another PX4-Autopilot SITL, use:
```bash
ros2 launch tilting_drone_x4 drone.launch.py drone_type:=gz_x500

```

### Run Offboard-mode Example
```bash
ros2 run tilting_drone_x4 offboard_control.py

```
To change the servo motor (rotor arm) angle:
```bash
ros2 param set /Drone_X4_Node arm_angle 30.0
```

### Run controller node
```bash
ros2 launch tilting_drone_x4 tilt_drone.launch.py

```
### TO edit gains:
```bash
tilting_drone_x4/config/controller/initial_gains_x500.yaml
```
K_P = Position gain <br/>
K_v = Velocity gain <br/>
K_R = Orientation gain <br/>
k_w = Angular velocity gain <br/>
<br/>
position = Desired Position in meter.<br/>
Eular = Desired Roll pitch yaw angle in rad.<br/>
