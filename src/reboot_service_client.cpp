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
#include <string>

class RebootServiceClient : public rclcpp::Node
{
public:
    RebootServiceClient()
    : Node("reboot_service_client"), has_rebooted_(false)
    {
        // Create service clients for left and right wheel reboot services
        left_client_ = this->create_client<std_srvs::srv::Trigger>("/left_wheel/reboot_service");
        right_client_ = this->create_client<std_srvs::srv::Trigger>("/right_wheel/reboot_service");

        RCLCPP_INFO(this->get_logger(), "RebootServiceClient initialized.");

        // タイマーの作成（100ms後にreboot_wheelsを実行）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RebootServiceClient::reboot_wheels, this)
        );
    }

private:
    // Service clients for left and right wheel reboot services
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_client_;

    // タイマー
    rclcpp::TimerBase::SharedPtr timer_;

    // 再起動が既に実行されたかどうかのフラグ
    bool has_rebooted_;

    /**
     * @brief Sends reboot requests to both left and right wheel services.
     */
    void reboot_wheels()
    {
        if (has_rebooted_) {
            return;
        }

        has_rebooted_ = true;

        RCLCPP_INFO(this->get_logger(), "Rebooting wheels...");

        // タイマーをキャンセルして、再起動を一度だけ実行
        timer_->cancel();

        // Wait for the left wheel reboot service to be available (maximum 5 seconds)
        if (!left_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Left wheel reboot service not available.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Left wheel reboot service is available.");

        // Wait for the right wheel reboot service to be available (maximum 5 seconds)
        if (!right_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Right wheel reboot service not available.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Right wheel reboot service is available.");

        // Create service requests for both wheels
        auto left_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto right_request = std::make_shared<std_srvs::srv::Trigger::Request>();

        // Send asynchronous service requests
        RCLCPP_INFO(this->get_logger(), "Sending reboot request to left wheel.");
        auto left_future = left_client_->async_send_request(left_request);

        RCLCPP_INFO(this->get_logger(), "Sending reboot request to right wheel.");
        auto right_future = right_client_->async_send_request(right_request);

        // サービスレスポンスを待機
        RCLCPP_INFO(this->get_logger(), "Waiting for left wheel reboot response.");
        auto left_result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), left_future);
        RCLCPP_INFO(this->get_logger(), "Waiting for right wheel reboot response.");
        auto right_result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), right_future);

        // サービス呼び出し結果の確認
        if (left_result == rclcpp::FutureReturnCode::SUCCESS && right_result == rclcpp::FutureReturnCode::SUCCESS) {
            auto left_response = left_future.get();
            auto right_response = right_future.get();

            if (left_response->success && right_response->success) {
                RCLCPP_INFO(this->get_logger(), "Both wheels rebooted successfully: Left: %s, Right: %s",
                            left_response->message.c_str(), right_response->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to reboot wheels: Left: %s, Right: %s",
                             left_response->message.c_str(), right_response->message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service on one or both wheels.");
        }

        // ノードをシャットダウン
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the RebootServiceClient node
    auto node = std::make_shared<RebootServiceClient>();

    // Spin the node to process callbacks
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
