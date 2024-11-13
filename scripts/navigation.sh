#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Function to display error messages
error_exit() {
    echo "$1" 1>&2
    exit 1
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
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash || error_exit "Failed to source ros2_ws setup.bash"
    echo "Sourced ~/ros2_ws/install/setup.bash"
else
    error_exit "~/ros2_ws/install/setup.bash not found."
fi

# Source NeuraTruck workspace setup
if [ -f ~/neuratruck_ws/install/setup.bash ]; then
    source ~/neuratruck_ws/install/setup.bash || error_exit "Failed to source neuratruck_ws setup.bash"
    echo "Sourced ~/neuratruck_ws/install/setup.bash"
else
    error_exit "~/neuratruck_ws/install/setup.bash not found."
fi

# ======== Execute Startup Launch File ========
echo "Launching startup.launch.py..."
ros2 launch amr_slam_nav_core startup.launch.py || error_exit "Failed to launch startup.launch.py"

# ======== Delay ========
echo "Waiting for 2 seconds..."
sleep 2

# ======== Execute Navigation Launch File ========
echo "Launching navigation.launch.py..."
ros2 launch amr_slam_nav_core navigation.launch.py || error_exit "Failed to launch navigation.launch.py"
