#!/bin/bash
# Load environment
source /home/<your-username>/.bashrc
source /home/<your-username>/ros2_ws/install/setup.bash
source /home/<your-username>/neuratruck_ws/install/setup.bash

# Execute ROS 2 launch file
ros2 launch amr_slam_nav_core startup.launch.py

ros2 launch nav2_bringup bringup_launch.py  map:=$HOME/map/home_ts_corridor/my_saved_map.yaml params_file:=$HOME/neuratruck_ws/src/amr_slam_nav_core/config/nav2_params.yaml
