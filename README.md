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

# amr_slam_nav_core

## パッケージの説明
`amr_slam_nav_core` は、自律移動ロボット（AMR）のナビゲーションとSLAM（Simultaneous Localization and Mapping）機能のコアコンポーネントを提供するROS 2パッケージです。このパッケージは、ロボットのセンサーからのデータを統合し、環境のマッピングとロボットの自己位置推定を行います。

## 主な機能
* IMUデータの購読とフィルタリング
* オドメトリデータの計算と公開
* ホイールの接続状態の監視
* SLAM機能の実装（Cartographer）
* 通信インターフェースの提供（rosbridge_websocket）
* LiDARデータの処理とフィルタリング
* ホイールモータのリブートサービス

## startup.launch.pyについて
このlaunchファイルは、自律移動ロボットに必要なノードの起動を行います。以下のノードが含まれています：
* **micro_ros_agent**: 左右のハブホイールモータ制御用マイコンボード（M5Stack）2つと通信するためのノード
* **rosbridge_websocket**: スマホやタブレットなど、リモートGUIとソケット通信するためのノード
* **rplidar_node**: ロボットに搭載された2D LiDARのlaserscanデータをpublishするためのノード
* **laser_scan_filters**: 生のlaserscanデータから、ロボット筐体部の干渉を受けている部分をフィルタリングするノード
* **cartographer**: laserscanデータを元にSLAMを行うためのノード
* **odometry_publisher**: ホイールの速度データを基にオドメトリデータを計算し、publishするノード
* **connection_checker**: ホイールモータの接続状態を定期的にチェックするノード
* **raw_imu_subscriber**: IMUの生データを購読し、フィルタリングしてpublishするノード
* **reboot_service_client**: システムシグナル（例: Ctrl+C）を受け取った際に、左右のホイールモータをリブートするサービスを呼び出すノード

## セットアップ方法

### 1. ROS 2のlaunchファイルの起動スクリプトに実行権限を付与
```bash
chmod +x /path/to/amr_slam_nav_core/scripts/startup.sh
```

### 2. ROS2プログラム起動サービスの設定
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

## テスト方法（疑似データを使用）

### 1. Terminal1: Cartographerを起動
```bash
ros2 launch amr_slam_nav_core startup_cartographer.launch.py
```

### 2. Terminal2: 左ホイールの速度データをパブリッシュ
```bash
# 左ホイール (left_vel)
while true; do
    ros2 topic pub /left_vel geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: 'left_wheel'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 20
    sleep 0.05
done
```

### 3. Terminal3: 右ホイールの速度データをパブリッシュ
```bash
# 右ホイール (right_vel)
while true; do
    ros2 topic pub /right_vel geometry_msgs/msg/TwistStamped "{header: {stamp: now, frame_id: 'right_wheel'}, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 20
    sleep 0.05
done
```

### 4. Terminal4: IMUの加速度データをパブリッシュ
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

### 5. Terminal: laserscanの疑似データをパブリッシュ
```bash
sudo chmod +x ${this package}/scripts/laserscan_sample.sh
./scripts/laserscan_sample.sh
```

## パッケージのビルド
パッケージをビルドするには、ROS 2ワークスペースのルートディレクトリで以下のコマンドを実行します：
```bash
colcon build --packages-select amr_slam_nav_core
```

ビルド後、ワークスペースをソースします：
```bash
source install/setup.bash
```

## 使用方法

### ノードの説明
`amr_slam_nav_core` パッケージには、以下の主要なノードが含まれています。それぞれのノードの役割と動作について詳しく説明します。

#### 1. `raw_imu_subscriber`
* **役割**: IMU（慣性測定装置）の生データを購読し、フィルタリングして`/imu/data_qos`にパブリッシュします。
* **動作**:
  - `/imu/data_raw` トピックからIMUデータを受信。
  - 受信データに移動平均フィルタを適用してノイズを低減。
  - フィルタリング後のデータを `/imu/data_qos` トピックにパブリッシュ。

#### 2. `odometry_publisher`
* **役割**: 左右のホイールの速度データを基にオドメトリデータを計算し、`/odometry/odom_encoder`にパブリッシュします。
* **動作**:
  - `/left_wheel/velocity` と `/right_wheel/velocity` トピックから車輪の速度データを受信。
  - 受信した速度データからロボットの位置（x, y）と姿勢（theta）を計算。
  - 計算したオドメトリデータを `/odometry/odom_encoder` トピックにパブリッシュ。

#### 3. `connection_checker`
* **役割**: 左右のホイールモータの接続状態を定期的にチェックし、接続状況をログに記録します。
* **動作**:
  - 定期的に `/connection_check_request` トピックに接続確認メッセージをパブリッシュ。
  - `/left_wheel/connection_response` と `/right_wheel/connection_response` トピックから接続状態のレスポンスを受信。
  - レスポンスに基づき、接続が確立されているかどうかをログに記録。

#### 4. `reboot_service_client`
* **役割**: システムシグナル（例: Ctrl+C）を受け取った際に、左右のホイールモータをリブートするサービスを呼び出します。
* **動作**:
  - シグナルを受信すると、`/left_wheel/reboot_service` と `/right_wheel/reboot_service` サービスを非同期に呼び出します。
  - サービスのレスポンスを受け取り、リブートの成功・失敗をログに記録。
  - 両サービスの呼び出しが完了した後、ROS 2 を安全にシャットダウンします。

### サービスの利用

リブートサービスを手動で呼び出す場合は、以下のコマンドを使用します：

```bash
ros2 service call /left_wheel/reboot_service std_srvs/srv/Trigger
ros2 service call /right_wheel/reboot_service std_srvs/srv/Trigger
```

## ディレクトリ構成
以下は、`amr_slam_nav_core`パッケージのディレクトリ構成です：

amr_slam_nav_core/
├── CMakeLists.txt
├── LICENSE
├── README.md
├── config
│   ├── config.lua
│   ├── ekf_config.yaml
│   ├── laser_filter_config.yaml
│   └── nav2_params.yaml
├── include
│   └── amr_slam_nav_core
├── launch
│   └── startup_cartographer.launch.py
├── package.xml
├── scripts
│   ├── laserscan_sample.sh
│   ├── startup.py
│   └── startup.sh
├── src
│   ├── connection_checker.cpp
│   ├── odometry_publisher.cpp
│   ├── raw_imu_subscriber.cpp
│   └── reboot_service_client.cpp
└── urdf
    └── n_v1.urdf

## 開発者向け

### 依存関係
このパッケージは以下の依存関係を持ちます：

* rclcpp
* sensor_msgs
* geometry_msgs
* nav_msgs
* tf2_ros
* rosbridge_server
* rplidar_node
* cartographer_ros

### コントリビューション
バグ報告や機能追加の提案は、GitHubリポジトリのIssuesセクションで受け付けています。プルリクエストも歓迎します。

### ライセンス
このプロジェクトはApache License 2.0の下でライセンスされています。詳細については、`LICENSE`ファイルを参照してください。

## トラブルシューティング

### ノードが起動しない:
* 必要な依存パッケージがインストールされているか確認してください。
* ワークスペースが正しくビルドされているか確認してください。

### センサーからデータが取得できない:
* センサーが正しく接続されているか、トピックが正しいか確認してください。
* `ros2 topic list`コマンドでデータがpublishされているか確認してください。

### サービスが動作しない:
* `reboot_service_client`ノードが正しく起動しているか確認してください。
* サービスが適切に登録されているか、`ros2 service list`で確認してください。

### その他:
* ログメッセージを確認し、エラーや警告を基に問題を特定してください。
* 必要に応じて、各ノードのデバッグモードを有効にして詳細なログを取得してください。