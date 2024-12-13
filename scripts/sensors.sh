#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Function to display error messages
error_exit() {
    echo "[ERROR] $1" 1>&2
    exit 1
}

# Trap SIGINT and execute cleanup
trap cleanup INT

# Function to source a workspace
source_workspace() {
    local workspace_dir=$1
    local setup_file="${workspace_dir}/install/setup.bash"

    if [ -f "${setup_file}" ]; then
        source "${setup_file}" || error_exit "Failed to source ${setup_file}"
        echo "[INFO] Sourced ${setup_file}"
    else
        error_exit "${setup_file} not found."
    fi
}

# ======== Load Environment ========
echo "[INFO] Loading environment variables..."

# Source ~/.bashrc if it exists
if [ -f ~/.bashrc ]; then
    source ~/.bashrc || error_exit "Failed to source ~/.bashrc"
    echo "[INFO] Sourced ~/.bashrc"
else
    error_exit "~/.bashrc not found."
fi

# Source ROS 2 workspace setup
ros2_ws_dir="${HOME}/ros2_ws"
source_workspace "${ros2_ws_dir}"

# ======== Check Environment Variables ========
if [ -z "${YOUR_CUSTOM_ROS2_WS}" ]; then
    error_exit "The environment variable YOUR_CUSTOM_ROS2_WS is not set. Please set it to the path of your custom ROS2 workspace."
fi

# Source NeuraTruck workspace setup
source_workspace "${YOUR_CUSTOM_ROS2_WS}"

# ======== Execute Startup Launch File ========
echo "[INFO] Launching startup.launch.py..."
ros2 launch amr_slam_nav_core sensors.launch.py
