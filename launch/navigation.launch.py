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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch引数の宣言
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map',
        description='Name of the map to load'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name = LaunchConfiguration('map_name')
    
    # マップ読み込みパスの定義
    home_dir = os.path.expanduser('~')
    map_dir = os.path.join(home_dir, 'maps')
    map_yaml = PathJoinSubstitution([
        TextSubstitution(text=map_dir + '/'),
        map_name,
        TextSubstitution(text='.yaml')
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
    
    return LaunchDescription([
        use_sim_time,
        map_name_arg,
        map_server_node,
    ])
