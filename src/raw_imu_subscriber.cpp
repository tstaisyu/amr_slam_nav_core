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
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Filter the incoming IMU data
        auto filtered_msg = filter_data(msg);

        // Change the frame ID to 'imu_link'
        filtered_msg->header.frame_id = "imu_link";

        // Publish the filtered IMU data
        publisher_->publish(*filtered_msg);
    }

    sensor_msgs::msg::Imu::SharedPtr filter_data(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Implement filtering logic here, such as noise reduction and bias correction
        // This example simply returns the original message
        return msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RawIMUSubscriber>());
    rclcpp::shutdown();
    return 0;
}
