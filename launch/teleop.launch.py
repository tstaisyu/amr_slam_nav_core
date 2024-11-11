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
    package_name = 'amr_slam_nav_core'
    package_dir = get_package_share_directory(package_name)
    config_file = os.path.join(package_dir, 'config', 'joy_teleop_config.yaml')

    # Node for Joy Node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[config_file],
        output='screen'
    )

    # Node for Teleop Twist Joy
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[config_file],
        remappings=[
            ('cmd_vel', 'cmd_vel')
        ],
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        teleop_twist_joy_node
    ])
