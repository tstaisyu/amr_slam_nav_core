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
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class HeartbeatNode : public rclcpp::Node
{
public:
    HeartbeatNode()
    : Node("heartbeat_node")
    {
        // Publisher: Jetson -> M5
        heartbeat_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/heartbeat", 10);
        // Subscriber: M5 -> Jetson
        heartbeat_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "/heartbeat_response",
            10,
            std::bind(&HeartbeatNode::heartbeat_response_callback, this, std::placeholders::_1)
        );

        // Timer for publishing heartbeat
        heartbeat_timer_ = this->create_wall_timer(
            1s,  // 1秒ごと
            std::bind(&HeartbeatNode::publish_heartbeat, this)
        );

        // Timer for checking heartbeat responses
        check_timer_ = this->create_wall_timer(
            1s,  // 1秒ごと
            std::bind(&HeartbeatNode::check_heartbeat_response, this)
        );

        last_response_time_ = this->now();
        has_received_response_ = false;

        RCLCPP_INFO(this->get_logger(), "HeartbeatNode has been started.");
    }

private:
    void publish_heartbeat()
    {
        auto message = std_msgs::msg::Int32();
        message.data = 1;  // ハートビート信号

        heartbeat_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published heartbeat signal.");
    }

    void heartbeat_response_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received heartbeat response: %d", msg->data);
        last_response_time_ = this->now();
        has_received_response_ = true;
    }

    void check_heartbeat_response()
    {
        auto current_time = this->now();
        auto elapsed_time = current_time - last_response_time_;

        if (elapsed_time > rclcpp::Duration(5s)) {  // 5秒のタイムアウト
            RCLCPP_WARN(this->get_logger(), "No heartbeat response received for %ld seconds.", elapsed_time.seconds());
            // 必要に応じてリカバリ処理をここに追加
        } else {
            RCLCPP_INFO(this->get_logger(), "Heartbeat is healthy. Last response %.2f seconds ago.", elapsed_time.seconds());
        }
    }

    // Publisher and Subscriber
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr heartbeat_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr heartbeat_subscriber_;

    // Timers
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr check_timer_;

    // Tracking last response time
    rclcpp::Time last_response_time_;
    bool has_received_response_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeartbeatNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
