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

// initial_pose_publisher_controller.cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

class InitialPosePublisherController : public rclcpp::Node
{
public:
    InitialPosePublisherController()
    : Node("initial_pose_publisher_controller")
    {
        // Subscribe to amcl_pose
        amcl_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose",
            10,
            std::bind(&InitialPosePublisherController::amclPoseCallback, this, std::placeholders::_1)
        );

        // Service to stop initial_pose_publisher
        stop_publisher_client_ = this->create_client<std_srvs::srv::Trigger>("stop_initial_pose_publisher");

        RCLCPP_INFO(this->get_logger(), "InitialPosePublisherController node initialized.");
    }

private:
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // 初期ポーズが設定されたとみなす条件を定義
        // 例えば、初期ポーズが設定された後の最初のamcl_poseを受信
        if (!initial_pose_set_)
        {
            RCLCPP_INFO(this->get_logger(), "Initial pose has been set. Sending stop signal to InitialPosePublisher.");
            initial_pose_set_ = true;

            // サービス呼び出しでInitialPosePublisherを停止
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            while (!stop_publisher_client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            auto result = stop_publisher_client_->async_send_request(request);
            // 結果を待たずに進む（非同期）
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_subscriber_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_publisher_client_;
    bool initial_pose_set_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisherController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
