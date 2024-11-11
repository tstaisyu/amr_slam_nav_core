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
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    urdf_file = os.path.join(package_dir, 'urdf', 'n_v1.urdf')

    # Paths of configuration files
    nav2_config = os.path.join(config_dir, 'nav2_params.yaml')
    laser_filters_config_path = os.path.join(config_dir, 'laser_filter_config.yaml')
    ekf_config = os.path.join(config_dir, 'ekf_config.yaml')
    urdf_file = os.path.join(urdf_dir, 'n_v1.urdf')

    # Load the robot's URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
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

    # Group of Sensor-related nodes
    sensor_nodes = GroupAction([
        # Node for connection check
        Node(
            package='amr_slam_nav_core',
            executable='connection_checker',
            name='connection_checker',
            output='screen'
        ),
        # Node for subscription of raw imu data
        raw_imu_subscriber = Node(
            package='amr_slam_nav_core',
            executable='raw_imu_subscriber',
            name='raw_imu_subscriber',
            output='screen',
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/data_qos', '/imu/data_qos')
            ]
        ),
        # Node for madgwick filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
                'publish_debug_topics': False,
                'gain': 0.1,
            }],
            remappings=[
                ('/imu/data_raw', '/imu/data_qos'),
                ('/imu/data', '/imu/data_filtered')
            ],
            output='screen'
        ),
        # Node for RPLiDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,
                'frame_id': 'laser_link'
            }],
            output='screen'
        ),
        # Node for laser scan filters
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_scan_filters',
            output='screen',
            parameters=[laser_filters_config_path],
        ),
    ])

    # Group of navigation related nodes
    navigation_nodes = GroupAction([
    # Node for odometry publisher
        Node(
            package='amr_slam_nav_core',
            executable='odometry_publisher',
            name='odometry_publisher',
            output='screen',
            remappings=[
                ('/odom', '/odometry/odom_encoder')
            ]
        ),
        # Node for robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # Node for EKF localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[
                ('/odom', '/odometry/odom_encoder'),
                ('/imu/data', '/imu/data_filtered')
            ]
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

    # Group of utility nodes
    utility_nodes = GroupAction([
        # Node for reboot service client
        Node(
            package='amr_slam_nav_core',
            executable='reboot_service_client',
            name='reboot_service_client',
            output='screen'
        ),
    ])

    # ======== Building a launch description ========
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('port', default_value='9090'),
        DeclareLaunchArgument('address', default_value=''),
        DeclareLaunchArgument('ssl', default_value='false'),
        DeclareLaunchArgument('certfile', default_value=''),
        DeclareLaunchArgument('keyfile', default_value=''),
        DeclareLaunchArgument('topics_glob', default_value=''),
        DeclareLaunchArgument('services_glob', default_value=''),
        DeclareLaunchArgument('params_glob', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Node groups
        micro_ros_agents,
        sensor_nodes,
        navigation_nodes,
        communication_nodes,
        utility_nodes,
    ])

