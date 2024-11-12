#!/bin/bash
# Load environment
source ~/.bashrc
source ~/ros2_ws/install/setup.bash
source ~/neuratruck_ws/install/setup.bash

# Execute ROS 2 launch file
ros2 launch amr_slam_nav_core startup.launch.py

delay=2

# Start mapping
ros2 launch amr_slam_nav_core navigation.launch.py

ros2 launch nav2_bringup bringup_launch.py