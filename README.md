<!-- Copyright 2024 Taisyu Shibata

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
-->

# Package explanation

## startup.launch.pyについて
このlaunchファイルは、自律移動ロボットに必要なノードの起動を行います。
* micro_ros_agent: 左右のハブホイールモータ制御用マイコンボード(M5Stack)2つと通信するためのノード
* rosbridge_websocket: スマホやタブレットなど、リモートGUIとソケット通信するためのノード
* rplidar_node: ロボットに搭載された2DLiDARのlaserscanデータをpublishするためのノード
* laser_scan_filters: 生のlaserscanデータから、ロボット筐体部の干渉を受けている部分をフィルタリングするノード
* cartographer: laserscanデータを元にSLAMするためのノード

# How to setting

1. ROS2のlaunchファイルの起動スクリプトに実行権限を付与
```bash
chmod +x /path/to/amr_slam_nav_core/scripts/startup.sh
```

2. ROS2プログラム起動サービスの設定
systemdサービスを設定することで、Jetsonが起動時に自動的にROS 2 launchファイルを実行します。
/etc/systemd/system/ディレクトリ内にros2-launch.serviceという名前でファイルを作成し、以下を記述します。

```bash
[Unit]
Description=Start ROS2 Launch
After=network.target

[Service]
Type=simple
User=<your-username>
# Environment="ROS_DOMAIN_ID=0" //as necessary
ExecStart=/home/username/ros2_ws/src/amr_slam_nav_core/scripts/startup.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

サービスの有効化と起動
```bash
sudo systemctl enable ros2-launch.service
sudo systemctl start ros2-launch.service
```

## How to test with pseudo data

1. Terminal1
```bash
ros2 launch amr_slam_nav_core startup_cartographer.launch.py
```

2. Terminal1
```bash
# 左ホイール (left_vel)
while true; do
    ros2 topic pub /left_vel geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: 'left_wheel'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 20
    sleep 0.05
done
```

3. Terminal1
```bash
# 右ホイール (right_vel)
while true; do
    ros2 topic pub /right_vel geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: 'right_wheel'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 20
    sleep 0.05
done
```

4. Terminal1
```bash
# IMUの加速度データに地球の重力を模擬
while true; do
    ros2 topic pub /imu/data_raw sensor_msgs/msg/Imu "{
        header: {stamp: now, frame_id: 'imu_link'},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0},
        angular_velocity: {x: 0.0, y: 0.0, z: 0.0},
        linear_acceleration: {x: 0.0, y: 0.0, z: 9.81}
    }" -r 20
    sleep 0.05
done
```

5. Terminal1
```bash
sudo chmod +x ${this package}/scripts/laserscan_sample.sh
./scripts/laserscan_sample.sh
```