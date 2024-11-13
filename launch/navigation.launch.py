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
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
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

    map_name_arg = LaunchConfiguration('map_name', default='map')
    
    # ======== Declaration of launch arguments ========   
    # マップ読み込みパスの定義
    home_dir = os.path.expanduser('~')
    map_dir = os.path.join(home_dir, 'maps')
    map_yaml = PathJoinSubstitution([
        map_dir,
        PythonExpression(["'", LaunchConfiguration('map_name'), ".yaml'"])
    ])
    
    # map_serverノードの定義
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml
        }]
    )

    # Node to manage lifecycle transitions
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server', 'recoveries_server', 'bt_navigator']
        }]
    )

    # Include the Nav2 bringup launch file for additional configurations
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': Command(['echo', map_yaml]),
            'use_sim_time': Command(['echo', use_sim_time]),
            'params_file': Command(['echo', os.path.join(nav2_params_dir, 'nav2_params.yaml')])
        }.items()
    )

    # ======== Building a launch description ========
    ld = LaunchDescription()

    # Add the actions to the launch description
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('map_name', default_value='map', description='Name of the map to save'))

    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(bringup_launch)

    return ld