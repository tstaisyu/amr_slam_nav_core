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

// Class for relaying Twist messages from one topic to another with QoS settings
class CmdVelRelay : public rclcpp::Node
{
public:
    CmdVelRelay()
    : Node("joy_cmd_vel_relay")
    {
        // Setup Quality of Service (QoS) settings for subscription
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

        // Subscribe to input_cmd_vel with the specified QoS profile
        subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "input_cmd_vel",
            qos,
            std::bind(&CmdVelRelay::listener_callback, this, std::placeholders::_1));

        // Publish to output_cmd_vel with the same QoS profile
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "output_cmd_vel",
            qos);
    }

private:
    // Callback function to relay the received Twist message to another topic
    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Publish the received message to output_cmd_vel
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Relayed cmd_vel message");
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the PoseSaver node
    rclcpp::spin(std::make_shared<PoseSaver>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
