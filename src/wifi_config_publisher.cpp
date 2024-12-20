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
#include <filesystem>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace fs = std::filesystem;

class WiFiConfigPublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the WiFiConfigPublisher node.
     * Initializes parameters, publisher, and sets up a timer to publish the WiFi configuration.
     */
    WiFiConfigPublisher()
    : Node("wifi_config_publisher")
    {
        initialize_parameters();
        initialize_publisher();
        initialize_timer();

        RCLCPP_INFO(this->get_logger(), "WiFiConfigPublisher node initialized.");
    }

private:
    /**
     * @brief Initializes the ROS 2 parameters with default values.
     * Declares 'ssid' and 'password' parameters and retrieves their values.
     */
    void initialize_parameters()
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("ssid", "default_ssid");
        this->declare_parameter<std::string>("password", "default_password");

        // Retrieve parameter values
        this->get_parameter("ssid", ssid_);
        this->get_parameter("password", password_);

        RCLCPP_INFO(this->get_logger(), "Parameters initialized: SSID='%s', PASSWORD='%s'", ssid_.c_str(), password_.c_str());
    }

    /**
     * @brief Initializes the publisher for sending WiFi configuration messages.
     * Sets up the publisher with a specified QoS profile.
     */
    void initialize_publisher()
    {
        // Create QoS settings (matching micro-ROS defaults)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // Create the publisher for 'wifi_config' topic
        wifi_config_publisher_ = this->create_publisher<std_msgs::msg::String>("wifi_config", qos);

        RCLCPP_INFO(this->get_logger(), "Publisher for 'wifi_config' topic created.");
    }

    /**
     * @brief Initializes a timer that triggers the publish_wifi_config method.
     * The timer is set to execute once after 1000 milliseconds (1 second).
     */
    void initialize_timer()
    {
        // Create a timer to publish WiFi config after 1 second
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() { this->publish_wifi_config(); }
        );

        RCLCPP_INFO(this->get_logger(), "Timer initialized to publish WiFi config after 1 second.");
    }

    /**
     * @brief Publishes the WiFi configuration to the 'wifi_config' topic.
     * Formats the SSID and password into a single string message.
     * Cancels the timer after publishing to ensure the message is sent only once.
     */
    void publish_wifi_config()
    {
        // Format the WiFi configuration message
        std::string message_data = "SSID:" + ssid_ + ";PASSWORD:" + password_;
        auto message = std_msgs::msg::String();
        message.data = message_data;

        // Log the message to be published
        RCLCPP_INFO(this->get_logger(), "Publishing WiFi config: '%s'", message.data.c_str());

        // Publish the WiFi configuration message
        wifi_config_publisher_->publish(message);

        // Cancel the timer to ensure the message is sent only once
        timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "WiFi config published and timer canceled.");
    }

    // Publisher for sending WiFi configuration messages
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wifi_config_publisher_;

    // Timer for scheduling the WiFi configuration message publication
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters for WiFi configuration
    std::string ssid_;
    std::string password_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the WiFiConfigPublisher node
    auto node = std::make_shared<WiFiConfigPublisher>();
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
