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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ======== Source the path and configuration file ========
    package_name = 'amr_slam_nav_core'
    package_dir = get_package_share_directory(package_name)
    config_dir = os.path.join(package_dir, 'config')
    cartographer_config_dir = config_dir
    cartographer_config_basename = 'config.lua'

    # ======== Declaration of launch arguments ========
    # Arguments for simulation time settings
    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Arguments for Cartographer settings    
    resolution = LaunchConfiguration(
        'resolution',
        default_value='0.05',
        description='Resolution of the occupancy grid'
    )
    publish_period_sec = LaunchConfiguration(
        'publish_period_sec',
        default_value='1.0',
        description='Publish period for occupancy grid'
    )

    # Arguments for map saving
    map_name_arg = LaunchConfiguration(
        'map_name',
        default_value='map',
        description='Name of the map to save'
    )
    
    map_name = LaunchConfiguration('map_name')

    # ======== Node definitions ========
    # Node for Cartographer
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename
        ],
        remappings=[
            ('/scan', '/scan_filtered'),
            ('/odom', '/odometry/filtered')
        ],
        output='screen'
    )

    # Node for Occupancy Grid
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': resolution,
            'publish_period_sec': publish_period_sec
        }],
        output='screen'
    )

    # Including a Launch file for saving maps
    save_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'save_map.launch.py')
        ),
        launch_arguments={'map_name': map_name}.items()
    )
    
    # An event handler that triggers saving the map when a Cartographer node is closed
    save_map_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=cartographer_node,
            on_exit=[save_map_launch]
        )
    )

    # ======== Building a launch description ========
    ld = LaunchDescription()

    # Launch arguments declaration
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('resolution', default_value='0.05', description='Resolution of the occupancy grid'))
    ld.add_action(DeclareLaunchArgument('publish_period_sec', default_value='1.0', description='Publish period for occupancy grid'))
    ld.add_action(DeclareLaunchArgument('map_name_arg', default_value='map', description='Name of the map to save'))

    # Add nodes to the launch description
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(save_map_on_exit)

    return ld
