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
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # デバイス権限の設定とmicroROS Agentの起動 (デバイス0)
        ExecuteProcess(
            cmd=['bash', '-c', 'sudo chmod 666 /dev/ttyACM0 && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6'],
            output='screen'
        ),
        # デバイス権限の設定とmicroROS Agentの起動 (デバイス1)
        ExecuteProcess(
            cmd=['bash', '-c', 'sudo chmod 666 /dev/ttyACM1 && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1 -v6'],
            output='screen'
        )
    ])

