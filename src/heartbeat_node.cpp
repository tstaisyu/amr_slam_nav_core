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
      last_response_time_left_(this->now()),
      last_response_time_right_(this->now()),
      left_wheel_healthy_(false),
      right_wheel_healthy_(false)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("heartbeat_topic", "/heartbeat");
        this->declare_parameter<std::string>("heartbeat_response_topic_left", "/left_wheel/heartbeat_response");
        this->declare_parameter<std::string>("heartbeat_response_topic_right", "/right_wheel/heartbeat_response");
        this->declare_parameter<int>("heartbeat_queue_size", 10);
        this->declare_parameter<int>("response_queue_size", 10);
        this->declare_parameter<int>("heartbeat_value", 1);
        this->declare_parameter<int>("heartbeat_response_value", 1);
        this->declare_parameter<double>("heartbeat_interval_seconds", 1.0);
        this->declare_parameter<double>("heartbeat_timeout_seconds", 5.0);

        this->get_parameter("heartbeat_topic", heartbeat_topic_);
        this->get_parameter("heartbeat_response_topic_left", heartbeat_response_topic_left_);
        this->get_parameter("heartbeat_response_topic_right", heartbeat_response_topic_right_);
        this->get_parameter("heartbeat_queue_size", heartbeat_queue_size_);
        this->get_parameter("response_queue_size", response_queue_size_);
        this->get_parameter("heartbeat_value", heartbeat_value_);
        this->get_parameter("heartbeat_response_value", heartbeat_response_value_);
        this->get_parameter("heartbeat_interval_seconds", heartbeat_interval_);
        this->get_parameter("heartbeat_timeout_seconds", heartbeat_timeout_);

        // Log parameter values
        RCLCPP_INFO(this->get_logger(), "Heartbeat Topic: %s", heartbeat_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Left Wheel Heartbeat Response Topic: %s", heartbeat_response_topic_left_.c_str());
        RCLCPP_INFO(this->get_logger(), "Right Wheel Heartbeat Response Topic: %s", heartbeat_response_topic_right_.c_str());
        RCLCPP_INFO(this->get_logger(), "Heartbeat Queue Size: %d", heartbeat_queue_size_);
        RCLCPP_INFO(this->get_logger(), "Response Queue Size: %d", response_queue_size_);
        RCLCPP_INFO(this->get_logger(), "Heartbeat Value: %d", heartbeat_value_);
        RCLCPP_INFO(this->get_logger(), "Heartbeat Response Value: %d", heartbeat_response_value_);
        RCLCPP_INFO(this->get_logger(), "Heartbeat Interval: %.2f seconds", heartbeat_interval_);
        RCLCPP_INFO(this->get_logger(), "Heartbeat Timeout: %.2f seconds", heartbeat_timeout_);

        // Define QoS profile with Best Effort reliability
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(heartbeat_queue_size_));
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // Create publisher for heartbeat
        heartbeat_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            heartbeat_topic_, qos_profile);

        // Create subscription for left wheel heartbeat responses
        heartbeat_subscriber_left_ = this->create_subscription<std_msgs::msg::Int32>(
            heartbeat_response_topic_left_,
            qos_profile,
            std::bind(&HeartbeatNode::heartbeat_response_callback_left, this, std::placeholders::_1)
        );

        // Create subscription for right wheel heartbeat responses
        heartbeat_subscriber_right_ = this->create_subscription<std_msgs::msg::Int32>(
            heartbeat_response_topic_right_,
            qos_profile,
            std::bind(&HeartbeatNode::heartbeat_response_callback_right, this, std::placeholders::_1)
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

    void heartbeat_response_callback_left(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received valid heartbeat response from LEFT wheel: %d", msg->data);
        last_response_time_left_ = this->now();

        if (!left_wheel_healthy_) {
            RCLCPP_INFO(this->get_logger(), "LEFT wheel is now healthy.");
            left_wheel_healthy_ = true;
        }
    }

    void heartbeat_response_callback_right(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received valid heartbeat response from RIGHT wheel: %d", msg->data);
        last_response_time_right_ = this->now();

        if (!right_wheel_healthy_) {
            RCLCPP_INFO(this->get_logger(), "RIGHT wheel is now healthy.");
            right_wheel_healthy_ = true;
        }
    }

    void check_heartbeat_response()
    {
        auto current_time = this->now();

        // Check left wheel heartbeat response
        auto elapsed_time_left = (current_time - last_response_time_left_).seconds();
        if (elapsed_time_left > heartbeat_timeout_) {
            if (left_wheel_healthy_) {
                RCLCPP_WARN(this->get_logger(), "No heartbeat response received from LEFT wheel for %.2f seconds.", elapsed_time_left);
                left_wheel_healthy_ = false;
            }
        }

        // Check right wheel heartbeat response
        auto elapsed_time_right = (current_time - last_response_time_right_).seconds();
        if (elapsed_time_right > heartbeat_timeout_) {
            if (right_wheel_healthy_) {
                RCLCPP_WARN(this->get_logger(), "No heartbeat response received from RIGHT wheel for %.2f seconds.", elapsed_time_right);
                right_wheel_healthy_ = false;
            }
        }
    }

    // Publisher and Subscriber
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr heartbeat_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr heartbeat_subscriber_left_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr heartbeat_subscriber_right_;

    // Timers
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr check_timer_;

    // Tracking last response time
    rclcpp::Time last_response_time_left_;
    rclcpp::Time last_response_time_right_;
    bool left_wheel_healthy_;
    bool right_wheel_healthy_;

    // Parameters
    std::string heartbeat_topic_;
    std::string heartbeat_response_topic_left_;
    std::string heartbeat_response_topic_right_;
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
