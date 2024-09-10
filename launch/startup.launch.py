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
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    port = LaunchConfiguration('port', default='9090')
    address = LaunchConfiguration('address', default='')
    ssl = LaunchConfiguration('ssl', default='false')
    certfile = LaunchConfiguration('certfile', default='')
    keyfile = LaunchConfiguration('keyfile', default='')
    topics_glob = LaunchConfiguration('topics_glob', default='')
    services_glob = LaunchConfiguration('services_glob', default='')
    params_glob = LaunchConfiguration('params_glob', default='')

    # microROSAgent node
    micro_ros_agent_acm0 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_acm0',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-v6'],
        output='screen'
    )

    micro_ros_agent_acm1 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_acm1',
        arguments=['serial', '--dev', '/dev/ttyACM1', '-v6'],
        output='screen'
    )

    # Rosbridge WebSocket node
    rosbridge_websocket_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'certfile': certfile,
            'keyfile': keyfile,
            'port': port,
            'address': address,
            'topics_glob': topics_glob,
            'services_glob': services_glob,
            'params_glob': params_glob,
        }],
        condition=IfCondition(ssl)
    )

    rosbridge_websocket_node_no_ssl = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': port,
            'address': address,
            'topics_glob': topics_glob,
            'services_glob': services_glob,
            'params_glob': params_glob,
        }],
        condition=UnlessCondition(ssl)
    )

    # Rosapi node
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        parameters=[{
            'topics_glob': topics_glob,
            'services_glob': services_glob,
            'params_glob': params_glob,
        }]
    )

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value=''),
        DeclareLaunchArgument('ssl', default_value='false'),
        DeclareLaunchArgument('certfile', default_value=''),
        DeclareLaunchArgument('keyfile', default_value=''),
        DeclareLaunchArgument('topics_glob', default_value=''),
        DeclareLaunchArgument('services_glob', default_value=''),
        DeclareLaunchArgument('params_glob', default_value=''),

        micro_ros_agent_acm0,
        micro_ros_agent_acm1,
        rosbridge_websocket_node,
        rosbridge_websocket_node_no_ssl,
        rosapi_node

    ])

