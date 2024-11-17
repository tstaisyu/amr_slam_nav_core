# Copyright 2024 Taisyu Shibata
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, JoinSubstitution, PythonExpression, Command, EnvironmentVariable, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ======== Source the path and configuration file ========
    package_name = 'amr_slam_nav_core'
    package_dir = get_package_share_directory(package_name)

    # Source the Nav2 package and configuration file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params_dir = os.path.join(package_dir, 'config')

    # Arguments for simulation time settings
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Arguments for map saving
    map_name_arg = LaunchConfiguration('map_name', default='map')
    
    # ======== Declaration of launch arguments ========   
    # Define the map file path
    home_dir = os.path.expanduser('~')
    robot_data_dir = os.path.join(home_dir, 'robot_data')
    pose_dir = os.path.join(robot_data_dir, 'pose')
    map_dir = os.path.join(robot_data_dir, 'maps')
    map_yaml = JoinSubstitution([
        map_dir,
        '/',
        LaunchConfiguration('map_name'),
        '.yaml'
    ])

    # Ensure directories exist
    ensure_directories = OpaqueFunction(function=ensure_directories_func)
    
    # Include the Nav2 bringup launch file for additional configurations
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(nav2_params_dir, 'nav2_params.yaml')
        }.items()
    )

    # Pose management nodes
    pose_saver_node = Node(
        package='amr_slam_nav_core',
        executable='pose_saver_nav',
        name='pose_saver_nav',
        output='screen'
    )

    initial_pose_publisher_node = Node(
        package='amr_slam_nav_core',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen'
    )

    # ======== Building a launch description ========
    # Create the launch description
    ld = LaunchDescription()

    # Add the actions to the launch description
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('map_name', default_value='map', description='Name of the map to save'))
    ld.add_action(ensure_directories)

    # Add the nodes to the launch description
    ld.add_action(bringup_launch)
    ld.add_action(pose_saver_node)
    ld.add_action(initial_pose_publisher_node)    

    return ld

def ensure_directories_func(context):
    home_dir = os.path.expanduser('~')
    robot_data_dir = os.path.join(home_dir, 'robot_data')
    pose_dir = os.path.join(robot_data_dir, 'pose')
    maps_dir = os.path.join(robot_data_dir, 'maps')
    
    # Create directories if they don't exist
    os.makedirs(pose_dir, exist_ok=True)
    os.makedirs(maps_dir, exist_ok=True)
    return [LogInfo(msg=f"Directories created: {pose_dir}, {maps_dir}")]