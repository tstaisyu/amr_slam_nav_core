#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Function to display error messages
error_exit() {
    echo "$1" 1>&2
    exit 1
}

# ======== Check Environment Variables ========
if [ -z "${YOUR_CUSTOM_ROS2_WS}" ]; then
    error_exit "The environment variable YOUR_CUSTOM_ROS2_WS is not set. Please set it to the path of your custom ROS2 workspace."
fi

# Cleanup function to kill background processes
cleanup() {
    echo "Cleaning up background processes..."
    if ps -p $mapping_pid > /dev/null 2>&1; then
        kill $mapping_pid || true
    fi
    if pgrep -f rosbridge_websocket > /dev/null 2>&1; then
        kill $(pgrep -f rosbridge_websocket) || true
    fi
    wait $mapping_pid 2>/dev/null || true
    wait $(pgrep -f rosbridge_websocket) 2>/dev/null || true
    echo "Background processes have been terminated."
}

# Trap SIGINT and execute cleanup
trap cleanup INT

# Function to source a workspace
source_workspace() {
    local workspace_dir=$1
    local setup_file="${workspace_dir}/install/setup.bash"

    if [ -f "${setup_file}" ]; then
        source "${setup_file}" || error_exit "Failed to source ${setup_file}"
        echo "Sourced ${setup_file}"
    else
        error_exit "${setup_file} not found."
    fi
}

# ======== Load Environment ========
echo "Loading environment variables..."

# Source ~/.bashrc if it exists
if [ -f ~/.bashrc ]; then
    source ~/.bashrc || error_exit "Failed to source ~/.bashrc"
    echo "Sourced ~/.bashrc"
else
    error_exit "~/.bashrc not found."
fi

# Source ROS 2 workspace setup
ros2_ws_dir="${HOME}/ros2_ws"
if [ -d "${ros2_ws_dir}" ]; then
    source_workspace "${ros2_ws_dir}"
else
    error_exit "~/ros2_ws/install/setup.bash not found."
fi

# Source NeuraTruck workspace setup
source_workspace "${YOUR_CUSTOM_ROS2_WS}"

# ======== Execute Mapping Launch File ========
echo "Launching mapping.launch.py..."
ros2 launch amr_slam_nav_core mapping.launch.py

# ======== Cleanup ========
cleanup