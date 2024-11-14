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

#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class ConnectionChecker : public rclcpp::Node
{
public:
  ConnectionChecker()
  : Node("connection_checker")
  {
    init_publisher(); // Initialize the publisher for connection checks
    init_subscriptions(); // Setup subscriptions to check connections for each wheel
    init_timer(); // Start a timer to periodically check connections
  }

private:
  // Initialize the publisher with the appropriate topic and QoS settings
  void init_publisher()
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("connection_check_request", 10);
  }

  // Helper function to log connection status based on received messages
  void check_connection(const std::shared_ptr<const std_msgs::msg::Int32> msg, const std::string &wheel_name)
  {
    if (msg->data == 1) {
      RCLCPP_INFO(this->get_logger(), "%s connected.", wheel_name.c_str());
      if (timer_) {
          timer_->cancel();  // Cancel the timer after a successful response
          timer_.reset();  // Optionally reset the timer to release resources
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "%s not connected.", wheel_name.c_str());
    }
  }

  // Set up subscriptions to wheel connection response topics
  void init_subscriptions()
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    left_wheel_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/left_wheel/connection_response", qos, 
      [this](const std::shared_ptr<const std_msgs::msg::Int32> msg) {
        this->check_connection(msg, "left_wheel");
      });
          
    right_wheel_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/right_wheel/connection_response", qos,
      [this](const std::shared_ptr<const std_msgs::msg::Int32> msg) {
        this->check_connection(msg, "right_wheel");
      });
  }

  // Initialize a timer to publish connection check requests
  void init_timer()
  {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this]() {
        std_msgs::msg::Int32 msg;
        msg.data = 1;
        publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published connection check request.");
      });
  }

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_wheel_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_wheel_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConnectionChecker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
