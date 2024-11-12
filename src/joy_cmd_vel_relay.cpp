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
#include "geometry_msgs/msg/twist.hpp"

class CmdVelRelay : public rclcpp::Node
{
public:
    CmdVelRelay()
    : Node("joy_cmd_vel_relay")
    {
        // 購読するトピックのQoS設定
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "joy_cmd_vel",
            qos,
            std::bind(&CmdVelRelay::listener_callback, this, std::placeholders::_1));

        // 発行するトピックのQoS設定
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel",
            qos);
    }

private:
    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
