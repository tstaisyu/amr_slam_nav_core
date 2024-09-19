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
<!--
1. udevルールの設定
udevルールを設定することで、Jetsonが起動時に自動的にM5Stackのデバイスパーミッションを適切に設定します。
/etc/udev/rules.d/99-usb-serial.rulesというファイルを作成し、以下を記述します。（デバイスIDは実際の環境に合わせて適宜調整）

```bash
KERNEL=="ttyACM0", MODE="0666"
KERNEL=="ttyUSB0", MODE="0666"
```

ルールを適用するためにudevをリロードします。

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
-->

1. ROS2のlaunchファイルの起動スクリプトに実行権限を付与
```bash
chmod +x /path/to/startup.sh
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
Environment="HOME=/home/<your-username>"
# Environment="ROS_DOMAIN_ID=0" //as necessary
ExecStart=/home/username/ros2_ws/src/amr_slam_nav_core/scripts/startup.sh


[Install]
WantedBy=multi-user.target
```

サービスの有効化と起動
```bash
sudo systemctl enable ros2-launch.service
sudo systemctl start ros2-launch.service
```
