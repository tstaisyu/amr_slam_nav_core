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
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class SensorSyncNode : public rclcpp::Node, public std::enable_shared_from_this<SensorSyncNode>
{
public:
    SensorSyncNode()
    : Node("sensor_sync_node")
    {
        // QoS settings for the subscriptions
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // Initialize the subscribers
        imu_sub_.subscribe(this, "/imu/data_raw", qos);
        odom_sub_.subscribe(this, "/odom", qos);
        scan_sub_.subscribe(this, "/scan", qos);
        left_wheel_sub_.subscribe(this, "/left_wheel/velocity", qos);
        right_wheel_sub_.subscribe(this, "/right_wheel/velocity", qos);

        // Set up ApproximateTimeSynchronizer
        sync_.reset(new Sync(MySyncPolicy(10), imu_sub_, odom_sub_, scan_sub_));
        sync_->registerCallback(std::bind(&SensorSyncNode::sync_callback, this, _1, _2, _3));

        // Set up the publishers for Cartographer
        carto_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/synchronized_imu", qos);
        carto_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/synchronized_odom", qos);
        carto_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/synchronized_scan", qos);

        RCLCPP_INFO(this->get_logger(), "SensorSyncNode initialized and subscribers set up.");
    }

private:
    void sync_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
                      const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
                      const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
                      const geometry_msgs::msg::TwistStamped::ConstSharedPtr left_wheel_msg,
                      const geometry_msgs::msg::TwistStamped::ConstSharedPtr right_wheel_msg)
    {
        // Calculate odometry based on wheel velocities
        nav_msgs::msg::Odometry calculated_odom = calculate_odometry(left_wheel_msg, right_wheel_msg);

        // Publish the synchronized data to Cartographer
        carto_imu_pub_->publish(*imu_msg);
        carto_odom_pub_->publish(*odom_msg);
        carto_scan_pub_->publish(*scan_msg);

        RCLCPP_INFO(this->get_logger(), "Synchronized IMU, Odometry, and LaserScan data received");
    }

    nav_msgs::msg::Odometry calculate_odometry(const geometry_msgs::msg::TwistStamped::ConstSharedPtr left_wheel_msg,
                                              const geometry_msgs::msg::TwistStamped::ConstSharedPtr right_wheel_msg)
    {
        // Odometry 計算のロジックをここに実装
        // 例: 差動駆動の基本的な Odometry 計算

        // 左右輪の速度から線形速度と角速度を計算
        double left_velocity = left_wheel_msg->twist.linear.x;
        double right_velocity = right_wheel_msg->twist.linear.x;

        double linear_velocity = (left_velocity + right_velocity) / 2.0;
        double angular_velocity = (right_velocity - left_velocity) / wheel_base_;

        // 時間差を計算（簡易的な例）
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // 前回の状態を保持（簡易的な例）
        x_ += linear_velocity * cos(theta_) * dt;
        y_ += linear_velocity * sin(theta_) * dt;
        theta_ += angular_velocity * dt;

        // Odometry メッセージを作成
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta_));

        odom.twist.twist.linear.x = linear_velocity;
        odom.twist.twist.angular.z = angular_velocity;

        return odom;
    }

    // Subscribers for IMU, Odometry, and LaserScan data
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped, SensorSyncNode> left_wheel_sub_;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped, SensorSyncNode> right_wheel_sub_;

    // Policy for ApproximateTimeSynchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu,
                                                            nav_msgs::msg::Odometry,
                                                            sensor_msgs::msg::LaserScan,
                                                            geometry_msgs::msg::TwistStamped,
                                                            geometry_msgs::msg::TwistStamped> MySyncPolicy;

    // ApproximateTimeSynchronizer for IMU, Odometry, and LaserScan data
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    // Publishers for synchronized data
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr carto_imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr carto_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr carto_scan_pub_;

    // Variables for odometry calculation
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    double wheel_base_ = 0.5;  // wheel base in meters
    rclcpp::Time last_time_ = this->now();
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
