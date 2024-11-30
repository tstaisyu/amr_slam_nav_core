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

cmake_minimum_required(VERSION 3.8)
project(amr_slam_nav_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options for GNU and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(message_filters)

# Optional dependency section
# find_package(<dependency> REQUIRED)

# Test configurations
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Enable copyright linter once copyright and license information is present
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(include)

# Declare a C++ executable
add_executable(raw_imu_subscriber src/raw_imu_subscriber.cpp)
add_executable(odometry_publisher src/odometry_publisher.cpp)
add_executable(connection_checker src/connection_checker.cpp)
add_executable(reboot_service_client src/reboot_service_client.cpp)
add_executable(joy_cmd_vel_relay src/joy_cmd_vel_relay.cpp)
add_executable(pose_publisher_node src/pose_publisher_node.cpp)
add_executable(pose_saver_nav src/pose_saver_nav.cpp)
add_executable(pose_saver_mapping src/pose_saver_mapping.cpp)
add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
add_executable(initial_pose_publisher_controller src/initial_pose_publisher_controller.cpp)
add_executable(wifi_config_publisher src/wifi_config_publisher.cpp)
add_executable(heartbeat_node src/heartbeat_node.cpp)
add_executable(sensor_sync_node src/sensor_sync_node.cpp)

# Specifying libraries for linking
ament_target_dependencies(raw_imu_subscriber rclcpp sensor_msgs tf2_geometry_msgs tf2)
ament_target_dependencies(odometry_publisher rclcpp geometry_msgs nav_msgs tf2_ros tf2 tf2_geometry_msgs)
ament_target_dependencies(connection_checker rclcpp std_msgs tf2_geometry_msgs)
ament_target_dependencies(reboot_service_client rclcpp rclcpp_lifecycle std_srvs tf2_geometry_msgs)
ament_target_dependencies(joy_cmd_vel_relay rclcpp geometry_msgs)
ament_target_dependencies(pose_publisher_node rclcpp geometry_msgs tf2_ros tf2_geometry_msgs)
ament_target_dependencies(pose_saver_nav rclcpp geometry_msgs)
ament_target_dependencies(pose_saver_mapping rclcpp geometry_msgs tf2_ros)
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs std_srvs)
ament_target_dependencies(initial_pose_publisher_controller rclcpp geometry_msgs std_srvs)
ament_target_dependencies(wifi_config_publisher rclcpp std_msgs)
ament_target_dependencies(heartbeat_node rclcpp std_msgs)
ament_target_dependencies(sensor_sync_node rclcpp sensor_msgs nav_msgs message_filters)

# Installation of targets and resources
install(TARGETS
  raw_imu_subscriber
  odometry_publisher
  connection_checker
  reboot_service_client
  joy_cmd_vel_relay
  pose_publisher_node
  pose_saver_nav
  pose_saver_mapping
  initial_pose_publisher
  initial_pose_publisher_controller
  wifi_config_publisher
  heartbeat_node
  sensor_sync_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
