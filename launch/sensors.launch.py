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
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ======== Declaration of launch arguments ========
    # Arguments for simulation time settings
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ======== Source the path and configuration file ========
    package_dir = get_package_share_directory('amr_slam_nav_core')
    config_dir = os.path.join(package_dir, 'config')
    urdf_dir = os.path.join(package_dir, 'urdf')

    # Paths of configuration files
    laser_filters_config_path = os.path.join(config_dir, 'laser_filter_config.yaml')
    ekf_config = os.path.join(config_dir, 'ekf_config.yaml')
    raw_imu_subscriber_config = os.path.join(config_dir, 'raw_imu_subscriber_params.yaml')
    urdf_file = os.path.join(urdf_dir, 'n_v1.urdf')

    # Load the robot's URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # ======== Node definitions ========
    # Group of Sensor-related nodes
    sensor_nodes = GroupAction([
        # Node for subscription of raw imu data
        Node(
            package='amr_slam_nav_core',
            executable='raw_imu_subscriber',
            name='raw_imu_subscriber',
            parameters=[raw_imu_subscriber_config],
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
        # Node for Static TF publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'base_footprint']
        ),
    ])

    # ======== Building a launch description ========
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'))

    # Add node groups
    ld.add_action(sensor_nodes)
    ld.add_action(navigation_nodes)

    return ld