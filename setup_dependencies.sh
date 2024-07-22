#!/bin/bash

# ANSI escape codes for colored text
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
NC='\033[0m' # No Color

echo "=========== Setting Up tilting_drone_x4 dependencies for PX4 SITL ============="

# Directory paths
WORKSPACE_DIR=$(dirname "$(dirname "$(realpath "$0")")")
MICRO_XRCE_DDS_AGENT_DIR="$WORKSPACE_DIR/../../Micro-XRCE-DDS-Agent"
PX4_AUTOPILOT_DIR="$WORKSPACE_DIR/../../PX4-Autopilot"
DRONE_X4_GAZEBO_DIR="$WORKSPACE_DIR/tilting_drone_x4/gazebo"
PX4_MODEL_DIR="$PX4_AUTOPILOT_DIR/Tools/simulation/gz/models"
AIRFRAME_DIR="$PX4_AUTOPILOT_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
SERVO_CPP_DIR="$PX4_AUTOPILOT_DIR/src/modules/simulation/gz_bridge"
PX4_MSGS_DIR="$WORKSPACE_DIR/px4_msgs"
AIRFRAME_CMAKE="$AIRFRAME_DIR/CMakeLists.txt"

# Function for printing info messages
print_info() {
    echo "[${GREEN}INFO${NC}] $1"
}

# Function for printing warning messages
print_warning() {
    echo "[${YELLOW}WARNING${NC}] $1"
}

# Function for printing error messages
print_error() {
    echo "[${RED}ERROR${NC}] $1"
}

# Function to clone and build a repository
clone_and_build() {
    local repo_url=$1
    local repo_dir=$2
    print_info "Cloning $repo_url into $repo_dir..."
    git clone "$repo_url" "$repo_dir" || { print_error "Failed to clone $repo_url"; exit 1; }
    cd "$repo_dir" || { print_error "Failed to cd into $repo_dir"; exit 1; }
    mkdir build
    cd build
    cmake ..
    make || { print_error "Failed to build $repo_dir"; exit 1; }
    sudo make install
    sudo ldconfig /usr/local/lib/
    cd "$WORKSPACE_DIR" || { print_error "Failed to return to workspace"; exit 1; }
}

# Clone and build Micro-XRCE-DDS-Agent if not already done
if [ ! -d "$MICRO_XRCE_DDS_AGENT_DIR" ]; then
    clone_and_build "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" "$MICRO_XRCE_DDS_AGENT_DIR"
else
    print_info "Micro-XRCE-DDS-Agent already exists."
fi


if [ ! -d "$PX4_AUTOPILOT_DIR" ]; then
    print_info "Cloning PX4-Autopilot into --> $PX4_AUTOPILOT_DIR..."
    git clone "https://github.com/PX4/PX4-Autopilot.git" --recursive "$PX4_AUTOPILOT_DIR" || { print_error "Failed to clone PX4-Autopilot"; exit 1; }
    cd "$PX4_AUTOPILOT_DIR" || { print_error "Failed to cd into PX4-Autopilot"; exit 1; }
    bash ./Tools/setup/ubuntu.sh || { print_error "Failed to run PX4 setup script"; exit 1; }
    cd "$WORKSPACE_DIR" || { print_error "Failed to return to workspace"; exit 1; }
else
    print_info "PX4-Autopilot already exists."
fi

# Clone px4_msgs if not already done
if [ ! -d "$PX4_MSGS_DIR" ]; then
    print_info "Cloning px4_msgs into --> $PX4_MSGS_DIR..."
    git clone "https://github.com/MayurRabadiya/px4_msgs.git" "$PX4_MSGS_DIR" || { print_error "Failed to clone px4_msgs"; exit 1; }
else
    print_info "px4_msgs already exists."
fi

# Copy tilting_drone_x4 model files to PX4-Autopilot if the directory exists
TILTING_DRONE_MODEL="$DRONE_X4_GAZEBO_DIR/models/tilting_drone_x4"
if [ -d "$TILTING_DRONE_MODEL" ]; then
    print_info "Copying tilting_drone_x4 gazebo files to --> $PX4_MODEL_DIR..."
    cp -r "$TILTING_DRONE_MODEL" "$PX4_MODEL_DIR" || { print_error "Failed to copy tilting_drone_x4 gazebo files"; exit 1; }
else
    print_error "tilting_drone_x4 directory does not exist. $TILTING_DRONE_MODEL"
fi

# Copy airframe file if it exists
AIRFRAME="$DRONE_X4_GAZEBO_DIR/airframe/7242_gz_tilting_drone_x4"
if [ -f "$AIRFRAME" ]; then
    print_info "Copying 7242_gz_tilting_drone_x4 to --> $AIRFRAME_DIR..."
    cp "$AIRFRAME" "$AIRFRAME_DIR" || { print_error "Failed to copy 7242_gz_tilting_drone_x4"; exit 1; }
else
    print_error "7242_gz_tilting_drone_x4 AirFrame file does not exist."
fi

# Copy Servo Interface Cpp file if it exists
SERVO_CPP_DRONE="$WORKSPACE_DIR/tilting_drone_x4/GZMixingInterfaceServo.cpp"
if [ -f "$SERVO_CPP_DRONE" ]; then
    print_info "Copying GZMixingInterfaceServo.cpp to  --> $SERVO_CPP_DIR..."
    cp "$SERVO_CPP_DRONE" "$SERVO_CPP_DIR" || { print_error "Failed to copy GZMixingInterfaceServo.cpp"; exit 1; }
else
    print_error "GZMixingInterfaceServo.cpp file does not exist."
fi

# Add 7242_gz_tilting_drone_x4 to px4_add_romfs_files in CMakeLists.txt if not already present
if grep -q "px4_add_romfs_files" "$AIRFRAME_CMAKE"; then
    if ! grep -q "7242_gz_tilting_drone_x4" "$AIRFRAME_CMAKE"; then
        print_info "Adding 7242_gz_tilting_drone_x4 to px4_add_romfs_files in $AIRFRAME_CMAKE..."
        sed -i '/px4_add_romfs_files/a \ \ \ \ 7242_gz_tilting_drone_x4' "$AIRFRAME_CMAKE" || { print_error "Failed to add 7242_gz_tilting_drone_x4 to $AIRFRAME_CMAKE"; exit 1; }
    else
        print_info "7242_gz_tilting_drone_x4 is already present in --> $AIRFRAME_CMAKE."
    fi
else
    print_error "px4_add_romfs_files function not found in $AIRFRAME_CMAKE."
fi

echo "=========== Set Up tilting_drone_x4 dependencies for PX4 SITL Finished ============="
