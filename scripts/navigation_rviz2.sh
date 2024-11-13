#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# ======== Set RMW_IMPLEMENTATION ========
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "Set RMW_IMPLEMENTATION to $RMW_IMPLEMENTATION"

# ======== Source the bashrc ========
source ~/.bashrc
echo "Sourced ~/.bashrc"

# ======== Launch the RViz2 and Navigation Launch Files ========
echo "Launching RViz for Navigation..."
ros2 launch nav2_bringup rviz_launch.py
