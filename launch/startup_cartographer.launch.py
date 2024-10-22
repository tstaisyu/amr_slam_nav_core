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
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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

    config_dir = os.path.join(get_package_share_directory('amr_slam_nav_core'), 'config')
    configuration_basename = 'config.lua'
    nav2_config = os.path.join(config_dir, 'nav2_params.yaml')
    laser_filters_config_path = os.path.join(config_dir, 'laser_filter_config.yaml')
    ekf_config = os.path.join(config_dir, 'ekf_config.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

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
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-v6'],
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

    # m5 connect initializer
    m5_connect_initializer = Node(
        package='amr_slam_nav_core',
        executable='m5_connect_initializer',
        name='m5_connect_initializer',
        output='screen'
    )

    # RPLiDAR node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 256000
        }],
        output='screen'
    )

    # Laser scan filters
    laser_scan_filters = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_scan_filters',
        output='screen',
        parameters=[laser_filters_config_path],
    )

    # EKF localization node
    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[{'/home/taisyu/neuratruck_ws/src/amr_slam_nav_core/config/ekf_config.yaml'}, {'use_sim_time': use_sim_time}],
        remappings=[
           # ('/odom', '/odom_cartographer'),
           # ('/imu/data', '/left_wheel_imu')  # Remap as necessary
        ]
    )

    # Static transform publishers
    static_transform_publisher_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_odom',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'map', 'odom'],
        output='screen'
    )

    static_transform_publisher_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom_to_base_link',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'odom', 'base_link'],
        output='screen'
    )

    static_transform_publisher_base_link_to_laser_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_link_to_laser_frame',
        arguments=['0.11', '0.08', '0.15', '0.0', '0.0', '1.0', '0.0', 'base_link', 'laser_frame'],
        output='screen'
    )

    static_transform_publisher_laser_frame_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_laser_frame_to_base_footprint',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'laser_frame', 'base_footprint'],
        output='screen'
    )

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=['-configuration_directory', config_dir, '-configuration_basename', configuration_basename],
        remappings=[('/scan', '/scan_filtered'),
                    ('/odom', '/odom_cartographer') 
        ],
        output='screen'
    )

    # Occupancy grid node
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

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),

        micro_ros_agent_acm0,
        micro_ros_agent_acm1,
        rosbridge_websocket_node,
        rosbridge_websocket_node_no_ssl,
        rosapi_node,
        m5_connect_initializer,
        rplidar_node,
        laser_scan_filters,
        ekf_localization_node,
        static_transform_publisher_map_to_odom,
        static_transform_publisher_odom_to_base_link,
        static_transform_publisher_base_link_to_laser_frame,
        static_transform_publisher_laser_frame_to_base_footprint,
        cartographer_node,
        occupancy_grid_node
    ])

