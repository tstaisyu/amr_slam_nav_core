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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def save_map(context, *args, **kwargs):
    try:
        # Launch引数からmap_nameを取得
        map_name = context.perform_substitution(LaunchConfiguration('map_name'))
        home_dir = os.path.expanduser('~')
        map_dir = os.path.join(home_dir, 'maps')

        # マップディレクトリの作成
        os.makedirs(map_dir, exist_ok=True)
        context.log.info(f"Created or verified map directory: {map_dir}")

        # pbstreamファイルのパス
        pbstream_file = os.path.join(map_dir, f'{map_name}.pbstream')
        context.log.info(f"pbstream file path: {pbstream_file}")

        # ROSマップファイルのパス
        ros_map_stem = os.path.join(map_dir, map_name)
        context.log.info(f"ROS map stem: {ros_map_stem}")

        # `/write_state` サービスの呼び出しコマンド
        # JSON形式の引数はダブルクオートで囲む必要があります
        call_write_state = [
            'ros2', 'service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState',
            f'{{"filename": "{pbstream_file}", "include_unfinished_submaps": true}}'
        ]

        # pbstreamからROSマップへの変換コマンド
        convert_pbstream = [
            'ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
            f'-map_filestem={ros_map_stem}',
            f'-pbstream_filename={pbstream_file}',
            '-resolution=0.05'
        ]

        # サービス呼び出しの実行
        context.log.info(f"Calling service to save pbstream: {pbstream_file}")
        ExecuteProcess(
            cmd=call_write_state,
            output='screen',
            shell=True
        ).execute(context)

        # サービス呼び出し後に変換プロセスを実行
        context.log.info(f"Converting pbstream to ROS map: {ros_map_stem}.yaml and {ros_map_stem}.pgm")
        ExecuteProcess(
            cmd=convert_pbstream,
            output='screen'
        ).execute(context)

        # マップ保存完了のログ
        context.log.info(f"Map saved as {ros_map_stem}.yaml and {ros_map_stem}.pgm in {map_dir}")

    except Exception as e:
        context.log.error(f"An error occurred while saving the map: {e}")

def generate_launch_description():
    ld = LaunchDescription()

    # Launch引数の宣言
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map',
        description='Name of the map to save'
    )
    ld.add_action(map_name_arg)

    # OpaqueFunctionを使用してマップ保存処理を実行
    save_map_action = OpaqueFunction(function=save_map)
    ld.add_action(save_map_action)

    return ld
