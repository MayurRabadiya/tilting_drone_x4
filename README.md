
# tilting_drone_x4

This repository contains a simulation model of a tilting arm quadcopter and a controller to control the drone. The model is based on PX4-Autopilot SITL.

## Minimum Requirements
- ROS Humble
- Gazebo Garden
- Ubuntu 22.04

## Setup Instructions

### Clone the Repository
To clone the repository, run the following commands in your terminal:

```bash
mkdir -p /drone_x4_ws/src
cd ~/drone_x4_ws/src/
git clone https://github.com/MayurRabadiya/tilting_drone_x4.git

```

### Setup Dependencies
```bash
cd ~/tilting_drone_x4
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
cd ~/drone_x4_ws
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
