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
`amr_slam_nav_core` は、自律移動ロボット（AMR）のナビゲーションとSLAM（Simultaneous Localization and Mapping）機能のコアコンポーネントを提供するROS 2パッケージです。このパッケージは、ロボットのセンサーからのデータを統合し、環境のマッピングとロボットの自己位置推定、およびロボットの制御を行います。

## 主な機能
* IMUデータの購読とフィルタリング：IMUセンサーからの生データをフィルタリングし、ノイズを低減したデータを提供します。
* オドメトリデータの計算とパブリッシュ：左右のホイールエンコーダーからのデータを使用して、ロボットの位置と姿勢を推定します。
* ホイールモータの接続状態の監視：ホイールモータとの通信状態を定期的にチェックし、接続が失われた場合にアラートを出します。
* SLAM機能の実装（Cartographer）：LiDARデータを使用してリアルタイムで環境の地図を作成します。
* 通信インターフェースの提供（rosbridge_websocket）：リモートデバイスとの通信を可能にし、遠隔操作やモニタリングをサポートします。
* LiDARデータの処理とフィルタリング：LiDARセンサーからのデータをフィルタリングし、不要なデータを除去します。
* ホイールモータのリブートサービス：モータのリブートをリモートから行うためのサービスを提供します。

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
- **ハートビート通信**：JetsonとM5間でのハートビート信号の送受信を行い、通信状態を監視します。

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

## パッケージのセットアップガイド

以下の手順に従って、`amr_slam_nav_core` パッケージを新規のROS 2ワークスペースにクローンし、ビルドしてください。

1. 事前準備

#### 必要なツールのインストール

ROS 2がインストールされていることを確認してください。また、以下のツールがインストールされていることを確認します。

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool git
```
#### rosdepの初期化
初めてrosdepを使用する場合は、以下のコマンドを実行します。

```bash
sudo rosdep init
rosdep update
```

2. ROS 2ワークスペースの作成
もしまだROS 2ワークスペースを作成していない場合は、以下のコマンドで作成します。既にワークスペースが存在する場合は、このステップをスキップしてください。
```bash
source ~/.bashrc
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

3. パッケージのクローン
`amr_slam_nav_core`と`micro_ros_agent` パッケージをROS 2ワークスペースの src ディレクトリにクローンします。

```bash
git clone https://github.com/tstaisyu/amr_slam_nav_core.git
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
```

4. 依存関係の解決
ワークスペースのルートディレクトリに戻り、rosdep を使用して依存関係をインストールします。

```bash
cd ~/ros2_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

## micro_ros_agentのビルド
`amr_slam_nav_core` パッケージをビルドする前に、`micro_ros_agent`をビルドします。
```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```

## amr_slam_nav_coreパッケージのビルド
以下の手順に従って、`amr_slam_nav_core` パッケージをビルドします。

### 前提条件
- ROS 2 がインストールされていること。
- Git がインストールされていること。
- 必要な依存パッケージがインストールされていること。

1. パッケージのビルド
colcon を使用してパッケージをビルドします。
```bash
colcon build --packages-select amr_slam_nav_core
```

※ 依存関係がある場合は、全体をビルドすることをお勧めします。
```bash
colcon build
```

2. ワークスペースのソース
ビルドが完了したら、ワークスペースをソースします。
```bash
source install/setup.bash
```
これで、amr_slam_nav_core パッケージがビルドされ、利用可能になります。

## 使用方法

### 起動スクリプトの実行権限を付与
スクリプトに実行権限を付与します。

```bash
chmod +x ~/ros2_ws/src/amr_slam_nav_core/scripts/*.sh
```
### ノードの起動
1. ロボットの起動（startup.launch.py）
ロボットの基本的なノードを起動します。
```bash
ros2 launch amr_slam_nav_core startup.launch.py
```
2. マッピングの開始（mapping.launch.py）
SLAMを実行し、環境のマッピングを開始します。
```bash
ros2 launch amr_slam_nav_core mapping.launch.py
```
3. ナビゲーションの開始（navigation.launch.py）
既存の地図を使用して、ロボットのナビゲーションを開始します。
```bash
ros2 launch amr_slam_nav_core navigation.launch.py map_name:=your_map_name
```
※ your_map_name は使用する地図の名前に置き換えてください。

### リモートマッピングスクリプトの実行
`mapping_remote.sh`
リモートPC上でマッピングとテレオペレーションを同時に開始するスクリプトです。環境変数 YOUR_CUSTOM_ROS2_WS を設定し、以下のコマンドで実行します。
```bash
export YOUR_CUSTOM_ROS2_WS=~/your_custom_ws
~/ros2_ws/src/amr_slam_nav_core/scripts/mapping_remote.sh
```
このスクリプトは以下の2つのLaunchファイルを同時に起動します。
* mapping_rviz2.launch.py
* teleop.launch.py

## マッピングとナビゲーションの手順
### マッピング
1. ロボットの基本ノードを起動します。
```bash
ros2 launch amr_slam_nav_core startup.launch.py
```
2. マッピングを開始します。
```bash
ros2 launch amr_slam_nav_core mapping.launch.py
```
3. リモートPC上でRVizを使用して地図を確認します。
```bash
ros2 launch amr_slam_nav_core mapping_rviz2.launch.py
```
4. マッピングが完了したら、地図を保存します。
```bash
ros2 launch amr_slam_nav_core save_map.launch.py map_name:=your_map_name
```

### ナビゲーション
ロボットの基本ノードを起動します。
1. 
```bash
ros2 launch amr_slam_nav_core startup.launch.py
```

2. ナビゲーションを開始します。
```bash
ros2 launch amr_slam_nav_core navigation.launch.py map_name:=your_map_name
```

3. リモートPC上でRVizを使用してロボットを制御します。
```bash
ros2 launch amr_slam_nav_core navigation_rviz2.launch.py
```

### ノードの説明
`amr_slam_nav_core` パッケージには、以下の主要なノードが含まれています。それぞれのノードの役割と動作について詳しく説明します。

#### 1. `raw_imu_subscriber`
* **役割**: IMU（慣性測定装置）の生データを購読し、フィルタリングして`/imu/data_qos`にパブリッシュします。
* **動作**:
  - `/imu/data_raw` トピックからIMUデータを受信。
  - 受信データに移動平均フィルタを適用してノイズを低減。
  - フィルタリング後のデータを `/imu/data_qos` トピックにパブリッシュ。
* **パラメータ**:
  - raw_imu_topic: 購読するIMUデータのトピック名（デフォルト: "imu/data_raw"）
  - filtered_imu_topic: パブリッシュするフィルタリング済みIMUデータのトピック名（デフォルト: "imu/data_qos"）
  - offset_x, offset_y, offset_z: IMUの位置オフセット（メートル）
  - yaw_offset: IMUの回転オフセット（ラジアン）

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

#### 5. `pose_publisher_node`
* **役割**: ロボットの現在位置を取得し、geometry_msgs::msg::PoseWithCovarianceStampedとしてパブリッシュします。
* **パラメータ**:
  - source_frame: トランスフォームのソースフレーム（デフォルト: "map"）
  - target_frame: トランスフォームのターゲットフレーム（デフォルト: "base_link"）
  - pose_topic: パブリッシュするトピック名（デフォルト: "pose_estimate"）
  - publish_rate_hz: パブリッシュレート（Hz）

#### 6. `heartbeat_node`
* **役割**: Jetsonデバイスとホイール制御を担当するM5Stackとの間でハートビートメッセージを交換し、通信状態をモニタリングします。このノードはシステムの健全性を確認し、M5Stackが適切に応答しているかを定期的にチェックすることで、ロボットの稼働状況を保証します。
* **パラメータ**:
  - heartbeat_publish_topic: ハートビートメッセージをパブリッシュするトピック名（デフォルト: "/heartbeat"）。
  - heartbeat_subscribe_topic: M5Stackからのハートビート応答を購読するトピック名（デフォルト: "/heartbeat_response"）。
  - heartbeat_publish_rate: ハートビートメッセージのパブリッシュ頻度（デフォルト: 1.0 秒ごと）。
  - heartbeat_timeout: 応答がない場合のタイムアウト期間（デフォルト: 5.0 秒）。この期間内に応答がない場合、接続が失われたと判断され、アラートが発生します。
* **動作**:
  - ハートビートのパブリッシュ: JetsonからM5Stackへ定期的にハートビート信号（通常は整数の 1）をパブリッシュします。
  - 応答の監視: M5Stackからの応答を購読し、設定されたタイムアウト期間内に応答があるか監視します。応答がタイムアウト期間内に受信できた場合は通信が正常であると判断し、そうでない場合は接続エラーがあると警告します。
  - エラーハンドリング: 応答がタイムアウトした場合、リカバリ処理をトリガーするか、運用者に警告を送信します。

### サービスの利用

リブートサービスを手動で呼び出す場合は、以下のコマンドを使用します：

```bash
ros2 service call /left_wheel/reboot_service std_srvs/srv/Trigger
ros2 service call /right_wheel/reboot_service std_srvs/srv/Trigger
```

## ディレクトリ構成
以下は、`amr_slam_nav_core`パッケージのディレクトリ構成です：

```plaintext
├── CMakeLists.txt
├── LICENSE
├── README.md
├── config
│   ├── config.lua
│   ├── ekf_config.yaml
│   ├── joy_teleop_config.yaml
│   ├── laser_filter_config.yaml
│   ├── nav2_params.yaml
│   └── raw_imu_subscriber_params.yaml
├── include
│   └── amr_slam_nav_core
├── launch
│   ├── mapping.launch.py
│   ├── mapping_rviz2.launch.py
│   ├── navigation.launch.py
│   ├── save_map.launch.py
│   ├── startup.launch.py
│   └── teleop.launch.py
├── package.xml
├── rviz
│   └── mapping.rviz
├── scripts
│   ├── laserscan_sample.sh
│   ├── mapping.sh
│   ├── mapping_remote.sh
│   ├── navigation.sh
│   ├── navigation_rviz2.sh
│   ├── save_map.sh
│   ├── startup.sh
│   └── wheel_reboot.sh
├── src
│   ├── connection_checker.cpp
│   ├── initial_pose_publisher.cpp
│   ├── initial_pose_publisher_controller.cpp
│   ├── joy_cmd_vel_relay.cpp
│   ├── odometry_publisher.cpp
│   ├── pose_publisher_node.cpp
│   ├── pose_saver_mapping.cpp
│   ├── pose_saver_nav.cpp
│   ├── raw_imu_subscriber.cpp
│   ├── reboot_service_client.cpp
│   └── wifi_config_publisher.cpp
│   └── heartbeat_node.cpp
└── urdf
    └── n_v1.urdf
```

## 開発者向け

### 依存関係
このパッケージは以下の依存関係を持ちます：

* rclcpp
* sensor_msgs
* geometry_msgs
* nav_msgs
* tf2_ros
* rosbridge_server
* rplidar_ros
* cartographer_ros
* nav2_bringup
* micro_ros_agent

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

## お問い合わせ
質問やサポートが必要な場合は、リポジトリのIssuesセクションまたはプルリクエストを通じてご連絡ください。