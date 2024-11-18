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
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

class InitialPosePublisherController : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the InitialPosePublisherController node.
     * Initializes subscribers and service clients.
     */
    InitialPosePublisherController()
    : Node("initial_pose_publisher_controller"), initial_pose_set_(false)
    {
        initialize_subscription();
        initialize_service_client();

        RCLCPP_INFO(this->get_logger(), "InitialPosePublisherController node initialized.");
    }

private:
    /**
     * @brief Initializes the subscriber to the 'amcl_pose' topic.
     * Subscribes with a queue size of 10 and binds the callback method.
     */
    void initialize_subscription()
    {
        amcl_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose",
            10,
            std::bind(&InitialPosePublisherController::amclPoseCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to 'amcl_pose' topic.");
    }

    /**
     * @brief Initializes the service client for 'stop_initial_pose_publisher'.
     * Creates a client for the Trigger service.
     */
    void initialize_service_client()
    {
        stop_publisher_client_ = this->create_client<std_srvs::srv::Trigger>("stop_initial_pose_publisher");
        RCLCPP_INFO(this->get_logger(), "Service client for 'stop_initial_pose_publisher' created.");
    }

    /**
     * @brief Callback function for the 'amcl_pose' subscription.
     * Detects when the initial pose is set and sends a request to stop the initial pose publisher.
     *
     * @param msg Shared pointer to the received PoseWithCovarianceStamped message.
     */
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Define the condition to consider the initial pose as set.
        // For example, consider the first received amcl_pose as the initial pose.
        if (!initial_pose_set_)
        {
            RCLCPP_INFO(this->get_logger(), "Initial pose has been set. Position: x=%.2f, y=%.2f, z=%.2f",
                        msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Sending stop signal to InitialPosePublisher.");
            initial_pose_set_ = true;

            // Create a Trigger service request to stop the InitialPosePublisher.
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

            // Wait for the service to be available with a timeout of 1 second.
            while (!stop_publisher_client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            // Send the service request asynchronously.
            auto result_future = stop_publisher_client_->async_send_request(request);

            // Optionally, handle the service response.
            // Here, we log the response once it's received.
            // This is done in a separate thread to avoid blocking the callback.
            std::thread([this, result_future = std::move(result_future)]() mutable {
                if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto response = result_future.get();
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Successfully stopped InitialPosePublisher: %s", response->message.c_str());
                    }
                    else {
                        RCLCPP_WARN(this->get_logger(), "Failed to stop InitialPosePublisher: %s", response->message.c_str());
                    }
                }
                else {
                    RCLCPP_ERROR(this->get_logger(), "Service call to 'stop_initial_pose_publisher' failed.");
                }
            }).detach();
        }
    }

    // Subscriber to the 'amcl_pose' topic
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscriber_;

    // Service client to stop the initial pose publisher
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_publisher_client_;

    // Flag to indicate whether the initial pose has been set
    bool initial_pose_set_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the InitialPosePublisherController node
    auto node = std::make_shared<InitialPosePublisherController>();
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
