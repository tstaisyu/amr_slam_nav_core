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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <memory>
#include <string>

using namespace rclcpp_lifecycle;

class RebootServiceClient : public LifecycleNode
{
public:
    RebootServiceClient()
    : LifecycleNode("reboot_service_client")
    {
        // Create service clients for left and right wheel reboot services
        left_client_ = this->create_client<std_srvs::srv::Trigger>("/left_wheel/reboot_service");
        right_client_ = this->create_client<std_srvs::srv::Trigger>("/right_wheel/reboot_service");

        RCLCPP_INFO(this->get_logger(), "RebootServiceClient initialized.");
    }

    LifecycleNodeInterface::CallbackReturn on_configure(const State &)
    {
        left_client_ = this->create_client<std_srvs::srv::Trigger>("/left_wheel/reboot_service");
        right_client_ = this->create_client<std_srvs::srv::Trigger>("/right_wheel/reboot_service");

        RCLCPP_INFO(this->get_logger(), "RebootServiceClient configured.");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_activate(const State &)
    {
        RCLCPP_INFO(this->get_logger(), "RebootServiceClient activated.");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_deactivate(const State &)
    {
        RCLCPP_INFO(this->get_logger(), "RebootServiceClient deactivated.");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_shutdown(const State &)
    {
        reboot_wheels();
        RCLCPP_INFO(this->get_logger(), "RebootServiceClient shut down.");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    // Service clients for left and right wheel reboot services
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_client_;

    /**
     * @brief Sends reboot requests to both left and right wheel services.
     */
    void reboot_wheels()
    {
        RCLCPP_INFO(this->get_logger(), "Rebooting wheels...");

        // Wait for the left wheel reboot service to be available (maximum 5 seconds)
        if (!left_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Left wheel reboot service not available.");
            return;
        }

        // Wait for the right wheel reboot service to be available (maximum 5 seconds)
        if (!right_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Right wheel reboot service not available.");
            return;
        }

        // Create service requests for both wheels
        auto left_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto right_request = std::make_shared<std_srvs::srv::Trigger::Request>();

        // Send asynchronous service requests
        auto left_future = left_client_->async_send_request(left_request);
        auto right_future = right_client_->async_send_request(right_request);

        // Wait for the service responses
        auto left_result = rclcpp::spin_until_future_complete(get_node_base_interface(), left_future);
        auto right_result = rclcpp::spin_until_future_complete(get_node_base_interface(), right_future);

        // Check the results of the service calls
        if (left_result == rclcpp::FutureReturnCode::SUCCESS && right_result == rclcpp::FutureReturnCode::SUCCESS) {
            if (left_future.get()->success && right_future.get()->success) {
                RCLCPP_INFO(this->get_logger(), "Both wheels rebooted successfully: Left: %s, Right: %s",
                            left_future.get()->message.c_str(), right_future.get()->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to reboot wheels: Left: %s, Right: %s",
                             left_future.get()->message.c_str(), right_future.get()->message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service on one or both wheels.");
        }
    }
};

int main(int argc, char ** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the RebootServiceClient node
    auto node = std::make_shared<RebootServiceClient>();

    rclcpp::spin_some(node->get_node_base_interface());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
