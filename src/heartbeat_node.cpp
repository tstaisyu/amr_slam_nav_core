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
    : Node("heartbeat_node"),
      last_response_time_(this->now()),
      has_received_response_(false)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("heartbeat_topic", "/heartbeat");
        this->declare_parameter<std::string>("heartbeat_response_topic", "/heartbeat_response");
        this->declare_parameter<int>("heartbeat_queue_size", 10);
        this->declare_parameter<int>("response_queue_size", 10);
        this->declare_parameter<int>("heartbeat_value", 1);
        this->declare_parameter<int>("heartbeat_response_value", 1);
        this->declare_parameter<double>("heartbeat_interval_seconds", 1.0);
        this->declare_parameter<double>("heartbeat_timeout_seconds", 5.0);

        this->get_parameter("heartbeat_topic", heartbeat_topic_);
        this->get_parameter("heartbeat_response_topic", heartbeat_response_topic_);
        this->get_parameter("heartbeat_queue_size", heartbeat_queue_size_);
        this->get_parameter("response_queue_size", response_queue_size_);
        this->get_parameter("heartbeat_value", heartbeat_value_);
        this->get_parameter("heartbeat_response_value", heartbeat_response_value_);
        this->get_parameter("heartbeat_interval_seconds", heartbeat_interval_);
        this->get_parameter("heartbeat_timeout_seconds", heartbeat_timeout_);

        // Log parameter values
        RCLCPP_INFO(this->get_logger(), "Heartbeat Topic: %s", heartbeat_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Heartbeat Response Topic: %s", heartbeat_response_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Heartbeat Queue Size: %d", heartbeat_queue_size_);
        RCLCPP_INFO(this->get_logger(), "Response Queue Size: %d", response_queue_size_);
        RCLCPP_INFO(this->get_logger(), "Heartbeat Value: %d", heartbeat_value_);
        RCLCPP_INFO(this->get_logger(), "Heartbeat Response Value: %d", heartbeat_response_value_);
        RCLCPP_INFO(this->get_logger(), "Heartbeat Interval: %.2f seconds", heartbeat_interval_);
        RCLCPP_INFO(this->get_logger(), "Heartbeat Timeout: %.2f seconds", heartbeat_timeout_);

        // Create publisher for heartbeat
        heartbeat_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            heartbeat_topic_, heartbeat_queue_size_);

        // Create subscription for heartbeat responses
        heartbeat_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            heartbeat_response_topic_,
            response_queue_size_,
            std::bind(&HeartbeatNode::heartbeat_response_callback, this, std::placeholders::_1)
        );

        // Create timer for publishing heartbeat
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(heartbeat_interval_),
            std::bind(&HeartbeatNode::publish_heartbeat, this)
        );

        // Timer for checking heartbeat responses
        check_timer_ = this->create_wall_timer(
            1s,
            std::bind(&HeartbeatNode::check_heartbeat_response, this)
        );

        RCLCPP_INFO(this->get_logger(), "HeartbeatNode has been started.");
    }

private:
    void publish_heartbeat()
    {
        auto message = std_msgs::msg::Int32();
        message.data = heartbeat_value_;    // Heartbeat signal

        heartbeat_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published heartbeat signal: %d", message.data);
    }

    void heartbeat_response_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (msg->data == heartbeat_response_value_) {
            RCLCPP_INFO(this->get_logger(), "Received valid heartbeat response: %d", msg->data);
            last_response_time_ = this->now();
            has_received_response_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Received unexpected heartbeat response: %d", msg->data);
        }
    }

    void check_heartbeat_response()
    {
        auto current_time = this->now();
        auto elapsed_time = (current_time - last_response_time_).seconds();

        if (has_received_response_ && elapsed_time > heartbeat_timeout_) {
            RCLCPP_WARN(this->get_logger(), "No heartbeat response received for %.2f seconds.", elapsed_time);
            // Recovery or alert mechanisms can be added here
            has_received_response_ = false;  // Reset flag after timeout
        } else {
            RCLCPP_INFO(this->get_logger(), "Heartbeat is healthy. Last response %.2f seconds ago.", elapsed_time);
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

    // Parameters
    std::string heartbeat_topic_;
    std::string heartbeat_response_topic_;
    int heartbeat_queue_size_;
    int response_queue_size_;
    int heartbeat_value_;
    int heartbeat_response_value_;
    double heartbeat_interval_;
    double heartbeat_timeout_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeartbeatNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
