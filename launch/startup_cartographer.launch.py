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
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, ExecuteProcess, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnShutdown

def publish_reboot(context, *args, **kwargs):
    import rclpy
    from std_msgs.msg import Int32
    import time

    # ROS 2の初期化
    rclpy.init(args=None)
    node = rclpy.create_node('shutdown_reboot_publisher')
    publisher = node.create_publisher(Int32, '/reboot', 10)

    # メッセージの作成とパブリッシュ
    msg = Int32()
    msg.data = 1
    publisher.publish(msg)
    node.get_logger().info('Reboot message published')

    # メッセージが送信されるのを待機
    end_time = time.time() + 2.0  # 2秒待機（必要に応じて調整）
    while rclpy.ok() and time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    # 追加の待機時間（必要に応じて）
    time.sleep(1.0)

    # ROS 2のシャットダウン
    rclpy.shutdown()

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

    urdf_file = os.path.join(get_package_share_directory('amr_slam_nav_core'), 'urdf', 'n_v1.urdf')
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # microROSAgent node
    micro_ros_agent_acm0 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_acm0',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-v4'],
        output='screen'
    )

    micro_ros_agent_acm1 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_acm1',
        arguments=['serial', '--dev', '/dev/ttyUSB1', '-v4'],
        parameters=[{'use_sim_time': False}],
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

    # raw_imu_subscriber
    raw_imu_subscriber = Node(
        package='amr_slam_nav_core',
        executable='raw_imu_subscriber',
        name='raw_imu_subscriber',
        output='screen',
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data_qos', '/imu/data_qos')
        ]
    )

    # madgwick_filter
    imu_filter_madgwick = Node(
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
    )

    # odometry publisher
    odometry_publisher = Node(
        package='amr_slam_nav_core',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen',
        remappings=[
            ('/odom', '/odometry/odom_encorder')                       # オドメトリのリマッピング
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # RPLiDAR node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 256000,
            'frame_id': 'laser_link'
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
        parameters=[ekf_config],
        # arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ('/odom', '/odometry/odom_encorder'),
            ('/imu/data', '/imu/data_filtered')  # Remap as necessary
        ]
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
                    ('/odom', '/odometry/filtered') 
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

    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=publish_reboot)
            ]
        )
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

#        micro_ros_agent_acm0,
#        micro_ros_agent_acm1,
#        rosbridge_websocket_node,
#        rosbridge_websocket_node_no_ssl,
#        rosapi_node,
        raw_imu_subscriber,
        imu_filter_madgwick,
        odometry_publisher,
        robot_state_publisher,
#        rplidar_node,
        laser_scan_filters,
        ekf_localization_node,
        cartographer_node,
        occupancy_grid_node,
#        shutdown_handler
    ])

