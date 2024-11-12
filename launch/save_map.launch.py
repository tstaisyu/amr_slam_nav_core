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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Launch引数の宣言
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map',
        description='Name of the map to save'
    )
    ld.add_action(map_name_arg)

    # Launch引数の取得
    map_name = LaunchConfiguration('map_name')

    # マップ保存先のディレクトリとファイルパスの設定
    home_dir = os.path.expanduser('~')
    map_dir = os.path.join(home_dir, 'maps')
    pbstream_file = os.path.join(map_dir, f'{map_name}.pbstream')
    ros_map_stem = os.path.join(map_dir, map_name)

    # マップディレクトリの作成
    mkdir = ExecuteProcess(
        cmd=['mkdir', '-p', map_dir],
        output='screen'
    )
    ld.add_action(mkdir)

    # `/write_state` サービスの呼び出し
    call_write_state = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call', '/write_state',
            'cartographer_ros_msgs/srv/WriteState',
            f'{{"filename": "{pbstream_file}", "include_unfinished_submaps": true}}'
        ],
        output='screen',
        shell=True
    )

    # pbstreamからROSマップへの変換コマンド
    convert_map = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
            f'-map_filestem={ros_map_stem}',
            f'-pbstream_filename={pbstream_file}',
            '-resolution=0.05'
        ],
        output='screen'
    )

    # サービス呼び出し完了後に変換プロセスを実行するイベントハンドラー
    save_map_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=call_write_state,
            on_exit=[convert_map, LogInfo(msg=[f'Map saved as {ros_map_stem}.yaml and {ros_map_stem}.pgm in {map_dir}'])]
        )
    )
    ld.add_action(call_write_state)
    ld.add_action(save_map_event_handler)

    return ld
