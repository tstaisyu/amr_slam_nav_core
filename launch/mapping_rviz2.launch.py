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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import IncludeLaunchDescription, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory of the package    
    package_name = 'amr_slam_nav_core'
    package_dir = get_package_share_directory(package_name)

    # Declare launch arguments
    rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.joinPathJoinSubstitution([package_dir, 'rviz', 'mapping.rviz']),
        description='Path to the RViz2 config file'
    )

    # Define the launch configuration for RViz
    rviz_config = LaunchConfiguration('rviz_config_file')

    # Define the RViz node with configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Build and return the launch description
    return LaunchDescription([
        rviz_config_file,
        rviz_node
    ])
