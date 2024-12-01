/* Copyright 2024 Taisyu Shibata
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class SensorSyncNode : public rclcpp::Node
{
public:
    SensorSyncNode()
    : Node("sensor_sync_node")
    {
        // QoS設定（必要に応じて調整）
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // message_filtersのサブスクライバーを初期化
        imu_sub_.subscribe(this, "/imu/data", qos);
        odom_sub_.subscribe(this, "/odom", qos);
        scan_sub_.subscribe(this, "/scan", qos);

        // ApproximateTimeSynchronizerの設定
        sync_.reset(new Sync(MySyncPolicy(10), imu_sub_, odom_sub_, scan_sub_));
        sync_->registerCallback(std::bind(&SensorSyncNode::sync_callback, this, _1, _2, _3));

        // 同期されたデータをCartographerに渡すためのパブリッシャーを設定
        carto_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/synchronized_imu", qos);
        carto_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/synchronized_odom", qos);
        carto_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/synchronized_scan", qos);
    }

private:
    void sync_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
                      const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
                      const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // 同期されたデータをパブリッシュ
        carto_imu_pub_->publish(*imu_msg);
        carto_odom_pub_->publish(*odom_msg);
        carto_scan_pub_->publish(*scan_msg);

        RCLCPP_INFO(this->get_logger(), "Synchronized IMU, Odometry, and LaserScan data received");
    }

    // サブスクライバーの宣言
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;

    // 同期ポリシーの定義
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu,
                                                            nav_msgs::msg::Odometry,
                                                            sensor_msgs::msg::LaserScan> MySyncPolicy;

    // Synchronizerの宣言
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    // Cartographerにデータをパブリッシュするパブリッシャー
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr carto_imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr carto_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr carto_scan_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
