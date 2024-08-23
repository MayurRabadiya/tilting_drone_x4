
# tilting_drone_x4

This repository contains a simulation model of a tilting arm quadcopter and a controller to control the drone. The model is based on PX4-Autopilot SITL.
The controller is implemeted inside the PX4-Autopilot firmware at low-level.

### branch feedback_control has same controller with high-level implementation.
[feedback_control](https://github.com/MayurRabadiya/tilting_drone_x4/tree/feedback_control).

## Minimum Requirements
- ROS Humble
- Gazebo Garden/Harmonic
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
cd workspace/drone_x4_ws/src
git clone https://github.com/MayurRabadiya/tilting_drone_x4.git
```

### Setup Dependencies
Default branch for drone_x4_px4 is "main" in dependency.
```bash
cd tilting_drone_x4
sh setup_dependencies.sh <branch-name-of-drone-x4-px4>
```
[drone_x4_px4 Repository](https://github.com/MayurRabadiya/drone_x4_px4.git).

This process will take some time. Once the setup is complete, the workspace structure will look like this:

    workspace
    ├── drone_x4_ws
    │   └── src
    │       ├── tilting_drone_x4
    │       └── px4_msgs
    ├── drone_x4_px4
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
Runs with empty environment.
```bash
ros2 launch tilting_drone_x4 drone_x4.launch.py
```
Run time parameter to chnage the world environment:
```bash
ros2 launch tilting_drone_x4 drone_x4.launch.py world:=_window
ros2 launch tilting_drone_x4 drone_x4.launch.py world:=_wind_turbine
```


### Modes
Mode 0: Manual mode. Control drone_x4 with manual input from parameter server.   <br/>
Mode 1: Infinity shape trajectory tracking. <br/>
Mode 2: Circular shape trajectory tracking. <br/>
Mode 3: Star shape trajectory tracking. <br/>

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

### To update gains:
##### Gains can be update with rqt_reconfiguration form ROS2 or in PX4 airframe file.
Note: IF ROS2 NODE IS RUNNING, THAN GAIN WILL BE UPDATED FROM rqt_reconfigure..
```bash
workspace/drone_x4_px4/ROMFS/px4fmu_common/init.d-posix/airframes/7242_gz_tilting_drone_x4
```
MC_KR*_GAIN = Rotation gain <br/>
MC_KA*_GAIN = Angular velocity gain <br/>
MC_KX*_GAIN = Position gain <br/>
MC_KV*_GAIN = velocity gain <br/>
<br/>

### To Debug GAZEBO:
## If drone model is not spawning in gazebo:
For this error:
```
ERROR [gz_bridge] Service call timed out. Check GZ_SIM_RESOURCE_PATH is set correctly.
ERROR [gz_bridge] Task start failed (-1)
ERROR [init] gz_bridge failed to start and spawn model
ERROR [px4] Startup script returned with return value: 256
```
Follow this steps:
In order to communicate successfully via DDS, the used network interface has to be multicast enabled. We’ve seen in past experiences that this might not necessarily be enabled by default (on Ubuntu or OSX) when using the loopback adapter. See the original issue or a conversation on ros-answers. You can verify that your current setup allows multicast with the ROS 2 tool:

In Terminal 1:
```bash
ros2 multicast receive
```
In Terminal 1:
```bash
ros2 multicast send
```
If the first command did not return a response similar to:
```bash
Received from xx.xxx.xxx.xx:43751: 'Hello World!'
```
then you will need to update your firewall configuration to allow multicast using ufw.
```bash
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```
You can check if the multicast flag is enabled for your network interface using the `ifconfig` tool and looking for MULITCAST in the flags section:
```bash
eno1: flags=4163<...,MULTICAST>
   ...
```
For more information: [troubleshooting](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#linux).

To kill the gazebo:
```bash
pkill -9 ruby
```
