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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the package name and retrieve its share directory
    package_name = 'amr_slam_nav_core'
    package_dir = get_package_share_directory(package_name)
    
    # Define the paths to the configuration files
    config_file = os.path.join(package_dir, 'config', 'joy_teleop_config.yaml')

    # Configure the Joy node to handle joystick inputs
    joy_node = Node(
        package='joy',  # Package containing the joy_node executable
        executable='joy_node',  # Executable name
        name='joy_node',  # Node name
        parameters=[config_file],  # Path to the Joy node configuration file
        output='screen',  # Output logs to the screen
        arguments=['--ros-args', '--log-level', 'WARN']  # Set log level to WARN
    )

    # Node for Teleop Twist Joy
    # Configure the Teleop Twist Joy node to convert joystick inputs to cmd_vel messages
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',  # Package containing the teleop_node executable
        executable='teleop_node',  # Executable name
        name='teleop_twist_joy',  # Node name
        parameters=[config_file],  # Path to the Teleop Twist Joy configuration file
        remappings=[
            ('cmd_vel', 'cmd_vel')  # Remap topics if necessary (currently redundant)
        ],
        output='screen',  # Output logs to the screen
        arguments=['--ros-args', '--log-level', 'INFO']  # Set log level to INFO
    )

    # Create and return the LaunchDescription with both nodes
    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node
    ])
