#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Function to display error messages
error_exit() {
    echo "[ERROR] $1" 1>&2
    exit 1
}

# ======== Load Environment ========
echo "[INFO] Loading environment variables..."

# ======== Set RMW_IMPLEMENTATION ========
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp || error_exit "Failed to set RMW_IMPLEMENTATION"
echo "[INFO] Set RMW_IMPLEMENTATION to $RMW_IMPLEMENTATION"

# ======== Source the bashrc ========
if [ -f ~/.bashrc ]; then
    source ~/.bashrc || error_exit "Failed to source ~/.bashrc"
    echo "[INFO] Sourced ~/.bashrc"
else
    error_exit "~/.bashrc not found."
fi

# ======== Launch the RViz2 and Navigation Launch Files ========
echo "[INFO] Launching RViz for Navigation..."
ros2 launch nav2_bringup rviz_launch.py || error_exit "Failed to launch RViz"
