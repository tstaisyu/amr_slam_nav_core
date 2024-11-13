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
#include "std_srvs/srv/trigger.hpp"
#include <memory>

class RebootServiceClient : public rclcpp::Node
{
public:
  RebootServiceClient()
  : Node("reboot_service_client")
  {
    left_client_ = this->create_client<std_srvs::srv::Trigger>("/left_wheel/reboot_service");
    right_client_ = this->create_client<std_srvs::srv::Trigger>("/right_wheel/reboot_service");
    global_node_handle = this->shared_from_this();
    // デストラクタでリブート処理を行う
  }

  ~RebootServiceClient()
  {
    if (rclcpp::ok()) {  // Ensure ROS 2 is still running
      reboot_wheels();
    }
  }

  // 共通のノードハンドルを登録
  static inline rclcpp::Node::SharedPtr global_node_handle = nullptr;

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_client_;

  void reboot_wheels()
  {
    auto left_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto right_request = std::make_shared<std_srvs::srv::Trigger::Request>();

    auto left_future = left_client_->async_send_request(left_request);
    auto right_future = right_client_->async_send_request(right_request);

    // Wait for both services to complete
    auto left_result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), left_future);
    auto right_result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), right_future);

    if (left_result == rclcpp::FutureReturnCode::SUCCESS && right_result == rclcpp::FutureReturnCode::SUCCESS) {
      if (left_future.get()->success && right_future.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Both wheels rebooted successfully: Left: %s, Right: %s",
                    left_future.get()->message.c_str(), right_future.get()->message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to reboot wheels: Left: %s, Right: %s",
                     left_future.get()->message.c_str(), right_future.get()->message.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service on one or both wheels.");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RebootServiceClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
