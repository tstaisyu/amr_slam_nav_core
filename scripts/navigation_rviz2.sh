#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Function to display error messages
error_exit() {
    echo "$1" 1>&2
    exit 1
}

# ======== Set RMW_IMPLEMENTATION ========
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp || error_exit "Failed to set RMW_IMPLEMENTATION"
echo "Set RMW_IMPLEMENTATION to $RMW_IMPLEMENTATION"

# ======== Source the bashrc ========
if [ -f ~/.bashrc ]; then
    source ~/.bashrc || error_exit "Failed to source ~/.bashrc"
    echo "Sourced ~/.bashrc"
else
    error_exit "~/.bashrc not found."
fi

# ======== Launch the RViz2 and Navigation Launch Files ========
echo "Launching RViz for Navigation..."
ros2 launch nav2_bringup rviz_launch.py || error_exit "Failed to launch RViz"
