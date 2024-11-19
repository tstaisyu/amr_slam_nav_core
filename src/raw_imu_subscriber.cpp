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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class RawIMUSubscriber : public rclcpp::Node
{
public:
    RawIMUSubscriber(): Node("raw_imu_subscriber") {
        // Set QoS to best effort for subscription to raw IMU data
        auto subscriber_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        subscriber_qos.best_effort();

        // Set QoS to reliable for publishing filtered IMU data
        auto publisher_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        publisher_qos.reliable();

        // Create subscription to raw IMU data
        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw",
            subscriber_qos,
            std::bind(&RawIMUSubscriber::imu_callback, this, std::placeholders::_1)
        );

        // Create publisher for filtered IMU data
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_qos",
            publisher_qos
        );

        // IMU offset values (Pose of the IMU frame relative to the base_link)
        offset_x_ = -0.045;  // x-axis offset
        offset_y_ = 0.105;   // y-axis offset
        offset_z_ = 0.10;    // z-axis offset

        // IMU yaw offset (rotation around the z-axis)
        yaw_offset_ = 0.0;
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Copy the incoming IMU data
        auto corrected_msg = *msg;

        // Implement filtering logic here
        correct_imu_data(corrected_msg);

        // Change the frame ID to 'imu_link'
        corrected_msg.header.frame_id = "imu_link";

        // Publish the filtered IMU data
        publisher_->publish(corrected_msg);
    }

    void correct_imu_data(sensor_msgs::msg::Imu &msg) {
        // IMUのオフセット位置ベクトル
        tf2::Vector3 r(offset_x_, offset_y_, offset_z_);

        // IMUの回転オフセットをクォータニオンに変換
        tf2::Quaternion q_offset;
        q_offset.setRPY(0.0, 0.0, yaw_offset_);

        // IMUデータのオリエンテーションを取得
        tf2::Quaternion q_msg;
        tf2::fromMsg(msg.orientation, q_msg);

        // オリエンテーションの補正
        tf2::Quaternion q_corrected = q_offset * q_msg;
        q_corrected.normalize();
        msg.orientation = tf2::toMsg(q_corrected);

        // 角速度の補正
        tf2::Vector3 omega(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
        tf2::Vector3 omega_corrected = tf2::quatRotate(q_offset, omega);
        msg.angular_velocity.x = omega_corrected.x();
        msg.angular_velocity.y = omega_corrected.y();
        msg.angular_velocity.z = omega_corrected.z();

        // 加速度の補正
        tf2::Vector3 acc(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

        // 遠心加速度の計算
        tf2::Vector3 centripetal_acc = omega_corrected.cross(omega_corrected.cross(r));

        // 加速度の補正
        tf2::Vector3 acc_corrected = acc - centripetal_acc;
        acc_corrected = tf2::quatRotate(q_offset, acc_corrected);

        msg.linear_acceleration.x = acc_corrected.x();
        msg.linear_acceleration.y = acc_corrected.y();
        msg.linear_acceleration.z = acc_corrected.z();
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

    // IMUのオフセット（メートル単位）
    double offset_x_;
    double offset_y_;
    double offset_z_;

    // IMUの回転オフセット（ラジアン単位）
    double yaw_offset_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RawIMUSubscriber>());
    rclcpp::shutdown();
    return 0;
}
