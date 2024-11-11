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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # マップ名を指定する引数の宣言
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map',
        description='保存するマップの名前'
    )
    
    map_name = LaunchConfiguration('map_name')
    
    # ホームディレクトリのパスを取得
    home_dir = os.path.expanduser('~')
    map_dir = os.path.join(home_dir, 'maps')
    
    # マップ保存ディレクトリを作成（存在しない場合）
    create_map_dir = ExecuteProcess(
        cmd=['mkdir', '-p', map_dir],
        output='screen'
    )
    
    # 保存コマンドの定義
    save_map_command = [
        'ros2', 'service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState',
        "{'filename': '" + map_dir + "/' + '" + map_name.perform(None) + "' + '.pbstream', 'include_unfinished_submaps': true}"
    ]
    
    save_map_process = ExecuteProcess(
        cmd=save_map_command,
        output='screen',
        shell=True
    )
    
    # マップ変換コマンドの定義
    convert_map_command = [
        'ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
        '-map_filestem=' + map_dir + '/${map_name}',
        '-pbstream_filename=' + map_dir + '/${map_name}.pbstream',
        '-resolution=0.05'
    ]
    
    convert_map_process = ExecuteProcess(
        cmd=convert_map_command,
        output='screen',
        shell=True
    )
    
    return LaunchDescription([
        map_name_arg,
        create_map_dir,
        save_map_process,
        convert_map_process
    ])
