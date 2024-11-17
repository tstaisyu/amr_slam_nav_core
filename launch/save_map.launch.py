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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, PythonExpression
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory of the package
    package_name = 'amr_slam_nav_core'
    package_dir = get_package_share_directory(package_name)

    # Declare and get the map name
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map',
        description='Name of the map to save'
    )
    map_name = LaunchConfiguration('map_name')

    # Define the directory to save the map
    map_directory = PathJoinSubstitution([os.path.expanduser('~'), 'robot_data', 'maps'])

    # Ensure the directory exists
    ensure_directory_exists = PythonExpression([
        "import os; os.makedirs('", map_directory, "', exist_ok=True)"
    ])

    # Define the command to save the map
    save_map_command = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', PathJoinSubstitution([map_directory, map_name])
        ],
        name='save_map',
        output='screen'
    )

    # Create and return the launch description
    ld = LaunchDescription([
        map_name_arg,
        ensure_directory_exists,
        save_map_command
    ])

    return ld