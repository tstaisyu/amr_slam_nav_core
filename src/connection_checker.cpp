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
    /**
     * @brief Constructor for the ConnectionChecker node.
     * Initializes publishers, subscriptions, and timers.
     */
    ConnectionChecker()
    : Node("connection_checker"), left_connected_(false), right_connected_(false)
    {
        initialize_publisher(); // Initialize the publisher for connection checks
        initialize_subscriptions(); // Setup subscriptions to check connections for each wheel
        initialize_timer(); // Start a timer to periodically check connections
    }

private:
    /**
     * @brief Initializes the publisher for sending connection check requests.
     * Uses a predefined QoS profile for best effort communication.
     */
    void initialize_publisher()
    {
        const rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                                           .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("connection_check_request", qos_profile);
        RCLCPP_INFO(this->get_logger(), "Publisher for 'connection_check_request' initialized.");
    }

    /**
     * @brief Initializes subscriptions to receive connection responses from wheels.
     * Subscribes to both left and right wheel response topics using the same QoS profile.
     */
    void initialize_subscriptions()
    {
        const rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                                           .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

       // Subscription for the left wheel connection response
        left_wheel_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/left_wheel/connection_response", qos_profile, 
            [this](const std::shared_ptr<const std_msgs::msg::Int32> msg) {
                handle_connection_response(msg, "left_wheel");
            });
        RCLCPP_INFO(this->get_logger(), "Subscribed to '/left_wheel/connection_response'.");
            
        // Subscription for the right wheel connection response
        right_wheel_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/right_wheel/connection_response", qos_profile,
            [this](const std::shared_ptr<const std_msgs::msg::Int32> msg) {
                handle_connection_response(msg, "right_wheel");
            });
        RCLCPP_INFO(this->get_logger(), "Subscribed to '/right_wheel/connection_response'.");
    }

    /**
     * @brief Initializes a timer that periodically publishes connection check requests.
     * The timer publishes a message every second to request connection status.
     */
    void initialize_timer()
    {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), [this]() {
                publish_connection_check();
         });
        RCLCPP_INFO(this->get_logger(), "Connection check timer initialized.");
    }

    /**
     * @brief Publishes a connection check request message.
     * Sends a std_msgs::msg::Int32 message with data set to 1.
     */
    void publish_connection_check()
    {
        std_msgs::msg::Int32 msg;
        msg.data = 1;
        publisher_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published connection check request.");
    }

    /**
     * @brief Handles incoming connection response messages from wheels.
     * Updates the connection status and cancels the timer if both wheels are connected.
     *
     * @param msg Shared pointer to the received Int32 message.
     * @param wheel_name Name of the wheel ("left_wheel" or "right_wheel").
     */
    void handle_connection_response(const std::shared_ptr<const std_msgs::msg::Int32> msg, const std::string &wheel_name)
    {
        if (msg->data == 1) {
            RCLCPP_INFO(this->get_logger(), "%s connected.", wheel_name.c_str());
            update_connection_status(wheel_name, true);
        } else {
            RCLCPP_WARN(this->get_logger(), "%s not connected.", wheel_name.c_str());
            update_connection_status(wheel_name, false);
        }
    }

    /**
     * @brief Updates the connection status of a specified wheel.
     * Cancels and resets the timer if both wheels are connected.
     *
     * @param wheel_name Name of the wheel ("left_wheel" or "right_wheel").
     * @param status Connection status (true for connected, false for disconnected).
     */
    void update_connection_status(const std::string &wheel_name, bool status)
    {
        if (wheel_name == "left_wheel") {
            left_connected_ = status;
        } else if (wheel_name == "right_wheel") {
            right_connected_ = status;
        }

        // Check if both wheels are connected
        if (left_connected_ && right_connected_) {
            if (timer_) {
                timer_->cancel();
                timer_.reset();
                RCLCPP_INFO(this->get_logger(), "Both wheels are connected. Timer stopped.");
            }
        }
    }

    // Publisher for sending connection check requests
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    // Subscriptions for receiving connection responses from left and right wheels
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_wheel_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_wheel_subscription_;

    // Timer for periodically publishing connection check requests
    rclcpp::TimerBase::SharedPtr timer_;

    // Flags to track the connection status of each wheel
    bool left_connected_;
    bool right_connected_;
};

int main(int argc, char ** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the ConnectionChecker node
    auto node = std::make_shared<ConnectionChecker>();
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
