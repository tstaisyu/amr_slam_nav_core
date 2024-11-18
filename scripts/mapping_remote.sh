#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Function to display an error message and exit
error_exit() {
    echo "[ERROR] $1" 1>&2
    exit 1
}

# Cleanup function: Terminates the launched background processes
cleanup() {
    echo "[INFO] Executing cleanup..."
    if ps -p $rviz_pid > /dev/null 2>&1; then
        kill $rviz_pid
        echo "[INFO] Terminated rviz2."
    fi

    if ps -p $teleop_pid > /dev/null 2>&1; then
        kill $teleop_pid
        echo "[INFO] Terminated teleop."
    fi

    exit 0
}

# Trap SIGINT (Ctrl-C) and execute cleanup
trap cleanup INT

# Function to source a workspace
source_workspace() {
    local workspace_dir=$1
    local setup_file="${workspace_dir}/install/setup.bash"

    if [ -f "${setup_file}" ]; then
        source "${setup_file}" || error_exit "Failed to source workspace: ${setup_file}"
        echo "[INFO] Sourced ${setup_file}"
    else
        error_exit "Workspace setup file not found: ${setup_file}"
    fi
}

# Load environment settings
echo "[INFO] Loading environment settings..."

# Source ~/.bashrc
if [ -f ~/.bashrc ]; then
    source ~/.bashrc || error_exit "Failed to source ~/.bashrc."
    echo "[INFO] Sourced ~/.bashrc"
else
    error_exit "~/.bashrc not found."
fi

# Source ROS2 main workspace
ros2_ws_dir="${HOME}/ros2_ws"
source_workspace "${ros2_ws_dir}"

# Check environment variables (add as needed)
if [ -z "${YOUR_CUSTOM_ROS2_WS}" ]; then
    error_exit "Environment variable YOUR_CUSTOM_ROS2_WS is not set. Please set the path to your custom ROS2 workspace."
fi

# Source custom ROS2 workspace
source_workspace "${YOUR_CUSTOM_ROS2_WS}"

# Execute Launch files
echo "[INFO] Launching mapping_rviz2.launch.py..."
ros2 launch amr_slam_nav_core mapping_rviz2.launch.py &
rviz_pid=$!
echo "[INFO] PID of mapping_rviz2.launch.py: $rviz_pid"

echo "[INFO] Launching teleop.launch.py..."
ros2 launch amr_slam_nav_core teleop.launch.py &
teleop_pid=$!
echo "[INFO] PID of teleop.launch.py: $teleop_pid"

# Monitor background processes
wait $rviz_pid
wait $teleop_pid

# Cleanup
cleanup
