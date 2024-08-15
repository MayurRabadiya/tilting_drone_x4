#!/bin/bash

# ANSI escape codes for colored text
RED='\033[31m'
GREEN='\033[32m'
YELLOW='\033[33m'
NC='\033[0m' # No Color

echo "=========== Setting Up tilting_drone_x4 dependencies for PX4 SITL ============="

# Directory paths
WORKSPACE_DIR=$(dirname "$(dirname "$(realpath "$0")")")
MICRO_XRCE_DDS_AGENT_DIR="$WORKSPACE_DIR/Micro-XRCE-DDS-Agent"
PX4_AUTOPILOT_DIR="$WORKSPACE_DIR/drone_x4_px4"
PX4_MSGS_DIR="$WORKSPACE_DIR/px4_msgs"

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
    print_info "Cloning drone_x4_px4 into --> $PX4_AUTOPILOT_DIR..."
    git clone "https://github.com/MayurRabadiya/drone_x4_px4.git" --recursive "$PX4_AUTOPILOT_DIR" || { print_error "Failed to clone drone_x4_px4"; exit 1; }
    cd "$PX4_AUTOPILOT_DIR" || { print_error "Failed to cd into drone_x4_px4"; exit 1; }
    bash ./Tools/setup/ubuntu.sh || { print_error "Failed to run PX4 setup script"; exit 1; }
    cd "$WORKSPACE_DIR" || { print_error "Failed to return to workspace"; exit 1; }
else
    print_info "drone_x4_px4 already exists."
fi

# Clone px4_msgs if not already done
if [ ! -d "$PX4_MSGS_DIR" ]; then
    print_info "Cloning px4_msgs into --> $PX4_MSGS_DIR..."
    git clone "https://github.com/MayurRabadiya/px4_msgs.git" "$PX4_MSGS_DIR" || { print_error "Failed to clone px4_msgs"; exit 1; }
else
    print_info "px4_msgs already exists."
fi


echo "=========== Set Up tilting_drone_x4 dependencies for PX4 SITL Finished ============="
