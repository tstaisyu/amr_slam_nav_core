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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # ament_add_pytest_test(test_micro_ros_agent test/test_micro_ros_agent.py)
endif()

add_executable(m5_connect_initializer src/m5_connect_initializer.cpp)

ament_target_dependencies(m5_connect_initializer rclcpp geometry_msgs)

install(TARGETS
  m5_connect_initializer
  DESTINATION lib/${PROJECT_NAME})

# Launch ファイルのインストール
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Config ファイルのインストール
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# URDF ファイルのインストール
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
