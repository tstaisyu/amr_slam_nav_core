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
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def save_map(context, *args, **kwargs):
    try:
        # Launch引数からmap_nameを取得し、文字列として解決
        map_name = context.perform_substitution(LaunchConfiguration('map_name'))
        home_dir = os.path.expanduser('~')
        map_dir = os.path.join(home_dir, 'maps')

        # マップディレクトリの作成
        os.makedirs(map_dir, exist_ok=True)
        print(f"Created or verified map directory: {map_dir}")

        # pbstreamファイルのパス
        pbstream_file = os.path.join(map_dir, f'{map_name}.pbstream')
        print(f"pbstream file path: {pbstream_file}")

        # ROSマップファイルのパス
        ros_map_stem = os.path.join(map_dir, map_name)
        print(f"ROS map stem: {ros_map_stem}")

        # `/write_state` サービスの呼び出しコマンド
        call_write_state_cmd = [
            f'ros2 service call /write_state '
            f'cartographer_ros_msgs/srv/WriteState '
            f'"{{\\"filename\\": \\"{pbstream_file}\\", \\"include_unfinished_submaps\\": true}}"'
        ]

        # サービス呼び出しの実行
        print(f"Calling service to save pbstream: {pbstream_file}")
        result = subprocess.run(call_write_state_cmd, shell=True, capture_output=True, text=True)

        if result.returncode != 0:
            print(f"Service call failed with return code {result.returncode}")
            print(f"stderr: {result.stderr}")
            return [LogInfo(msg=f"Service call failed: {result.stderr}")]
        else:
            print(f"Service call succeeded: {result.stdout}")

        # `.pbstream`ファイルが生成されたか確認
        if not os.path.exists(pbstream_file):
            print(f"pbstream file was not created: {pbstream_file}")
            return [LogInfo(msg=f"pbstream file was not created: {pbstream_file}")]
        elif os.path.getsize(pbstream_file) == 0:
            print(f"pbstream file is empty: {pbstream_file}")
            return [LogInfo(msg=f"pbstream file is empty: {pbstream_file}")]

        # pbstreamからROSマップへの変換コマンド
        convert_pbstream_cmd = [
            f'ros2 run cartographer_ros cartographer_pbstream_to_ros_map '
            f'-map_filestem={ros_map_stem} '
            f'-pbstream_filename={pbstream_file} '
            f'-resolution=0.05'
        ]

        # 変換プロセスの実行
        print(f"Converting pbstream to ROS map: {ros_map_stem}.yaml and {ros_map_stem}.pgm")
        result = subprocess.run(convert_pbstream_cmd, capture_output=True, text=True)

        if result.returncode != 0:
            print(f"Conversion failed with return code {result.returncode}")
            print(f"stderr: {result.stderr}")
            return [LogInfo(msg=f"Conversion failed: {result.stderr}")]
        else:
            print(f"Conversion succeeded: {result.stdout}")

        # `.yaml`と`.pgm`ファイルが生成されたか確認
        yaml_file = f"{ros_map_stem}.yaml"
        pgm_file = f"{ros_map_stem}.pgm"
        if not os.path.exists(yaml_file) or not os.path.exists(pgm_file):
            print(f"Conversion did not produce the expected files: {yaml_file}, {pgm_file}")
            return [LogInfo(msg=f"Conversion did not produce the expected files: {yaml_file}, {pgm_file}")]
        elif os.path.getsize(yaml_file) == 0 or os.path.getsize(pgm_file) == 0:
            print(f"One of the map files is empty: {yaml_file}, {pgm_file}")
            return [LogInfo(msg=f"One of the map files is empty: {yaml_file}, {pgm_file}")]

        # マップ保存完了のログ
        print(f"Map saved as {yaml_file} and {pgm_file} in {map_dir}")
        return [LogInfo(msg=f"Map saved as {yaml_file} and {pgm_file} in {map_dir}")]

    except Exception as e:
        print(f"An error occurred while saving the map: {e}")
        return [LogInfo(msg=f"An error occurred while saving the map: {e}")]

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
