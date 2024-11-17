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
#include <memory>
#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class RebootServiceClient : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the RebootServiceClient node.
     * Initializes service clients and sets up a timer to trigger reboot.
     */
    RebootServiceClient()
    : Node("reboot_service_client"), has_rebooted_(false)
    {
        // Create service clients for left and right wheel reboot services
        left_client_ = this->create_client<std_srvs::srv::Trigger>("/left_wheel/reboot_service");
        right_client_ = this->create_client<std_srvs::srv::Trigger>("/right_wheel/reboot_service");

        RCLCPP_INFO(this->get_logger(), "RebootServiceClient initialized.");

        // Create a timer to execute reboot_wheels after 100ms
        reboot_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RebootServiceClient::reboot_wheels, this)
        );
    }

private:
    /**
     * @brief Initiates the reboot process for both left and right wheels.
     * Sends asynchronous service requests and handles responses via callbacks.
     */
    void reboot_wheels()
    {
        if (has_rebooted_) {
            return;
        }

        has_rebooted_ = true;

        RCLCPP_INFO(this->get_logger(), "Rebooting wheels...");

        // Cancel the timer to ensure this method is only called once
        reboot_timer_->cancel();

        // Ensure the reboot services are available
        if (!left_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Left wheel reboot service not available.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Left wheel reboot service is available.");

        if (!right_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Right wheel reboot service not available.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Right wheel reboot service is available.");

        // Create service requests for both wheels
        auto left_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto right_request = std::make_shared<std_srvs::srv::Trigger::Request>();

        // Send asynchronous service requests with response callbacks
        RCLCPP_INFO(this->get_logger(), "Sending reboot request to left wheel.");
        left_client_->async_send_request(left_request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_response) {
                handle_reboot_response(future_response, "Left");
            });

        RCLCPP_INFO(this->get_logger(), "Sending reboot request to right wheel.");
        right_client_->async_send_request(right_request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_response) {
                handle_reboot_response(future_response, "Right");
            });
    }


    /**
     * @brief Callback function to handle the response from reboot service requests.
     * Logs the outcome and shuts down ROS if both reboots are successful.
     *
     * @param future_response The future response from the service call.
     * @param wheel_side Identifier for the wheel ("Left" or "Right").
     */
    void handle_reboot_response(
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_response,
        const std::string &wheel_side)
    {
        try {
            auto response = future_response.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "%s wheel rebooted successfully: %s", wheel_side.c_str(), response->message.c_str());
                reboot_success_[wheel_side] = true;
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "Failed to reboot %s wheel: %s", wheel_side.c_str(), response->message.c_str());
                reboot_success_[wheel_side] = false;
            }
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "Exception while getting response for %s wheel reboot: %s", wheel_side.c_str(), ex.what());
            reboot_success_[wheel_side] = false;
        }

        // Check if both reboots have been attempted
        if (reboot_success_.size() == 2) {
            if (reboot_success_["Left"] && reboot_success_["Right"]) {
                RCLCPP_INFO(this->get_logger(), "Both wheels rebooted successfully. Shutting down.");
            }
            else {
                RCLCPP_WARN(this->get_logger(), "One or both wheels failed to reboot.");
            }
            // Shutdown the node after handling the responses
            rclcpp::shutdown();
        }
    }

    // Service clients for left and right wheel reboot services
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_client_;

    // Timer for initiating the reboot process
    rclcpp::TimerBase::SharedPtr reboot_timer_;

    // Flag to ensure reboot is only attempted once
    bool has_rebooted_;

    // Map to track reboot success for each wheel
    std::unordered_map<std::string, bool> reboot_success_;
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
