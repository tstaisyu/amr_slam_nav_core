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
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ======== Declaration of launch arguments ========
    # Arguments for communication settings
    port = LaunchConfiguration('port', default='9090')
    address = LaunchConfiguration('address', default='')
    ssl = LaunchConfiguration('ssl', default='false')
    certfile = LaunchConfiguration('certfile', default='')
    keyfile = LaunchConfiguration('keyfile', default='')
    topics_glob = LaunchConfiguration('topics_glob', default='')
    services_glob = LaunchConfiguration('services_glob', default='')
    params_glob = LaunchConfiguration('params_glob', default='')

    # Arguments for simulation time settings
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ======== Source the path and configuration file ========
    package_dir = get_package_share_directory('amr_slam_nav_core')
    config_dir = os.path.join(package_dir, 'config')
    urdf_dir = os.path.join(package_dir, 'urdf')

    # Paths of configuration files
    heartbeat_config = os.path.join(config_dir, 'heartbeat_params.yaml')
    
    # ======== Node definitions ========

    # Group of Micro-ROS-Agent nodes
    micro_ros_agents = GroupAction([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='right_wheel_micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0', '-v4'],
            output='screen'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='left_wheel_micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB1', '-v4'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])

    # Group of M5Stack connection nodes
    m5stack_nodes = GroupAction([
        # Node for connection check
        Node(
            package='amr_slam_nav_core',
            executable='connection_checker',
            name='connection_checker',
            output='screen'
        ),
        # Node for heartbeat node
        Node(
            package='amr_slam_nav_core',
            executable='heartbeat_node',
            name='heartbeat_node',
            parameters=[heartbeat_config],
            output='screen'
        ),
    ])

    # Group of communication related nodes
    communication_nodes = GroupAction([
        # Node for rosbridge websocket (SSL)
        Node(
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
        ),
        # Node for rosbridge websocket (non-SSL)
        Node(
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
        ),
        # Node for rosapi
        Node(
            package='rosapi',
            executable='rosapi_node',
            parameters=[{
                'topics_glob': topics_glob,
                'services_glob': services_glob,
                'params_glob': params_glob,
            }]
        ),
    ])

    # ======== Building a launch description ========
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('port', default_value='9090', description='Port for rosbridge websocket'))
    ld.add_action(DeclareLaunchArgument('address', default_value='', description='Address for rosbridge websocket'))
    ld.add_action(DeclareLaunchArgument('ssl', default_value='false', description='Enable SSL for rosbridge websocket'))
    ld.add_action(DeclareLaunchArgument('certfile', default_value='', description='SSL certificate file for rosbridge websocket'))
    ld.add_action(DeclareLaunchArgument('keyfile', default_value='', description='SSL key file for rosbridge websocket'))
    ld.add_action(DeclareLaunchArgument('topics_glob', default_value='', description='Topic patterns to include in rosbridge'))
    ld.add_action(DeclareLaunchArgument('services_glob', default_value='', description='Service patterns to include in rosbridge'))
    ld.add_action(DeclareLaunchArgument('params_glob', default_value='', description='Parameter patterns to include in rosbridge'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'))

    # Add node groups
    ld.add_action(micro_ros_agents)
    ld.add_action(m5stack_nodes)
    ld.add_action(communication_nodes)

    return ld
