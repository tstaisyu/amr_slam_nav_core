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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <cstdlib> // For std::getenv
#include <string>
#include <filesystem> // For std::filesystem::exists

namespace fs = std::filesystem;

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher() 
    : Node("initial_pose_publisher"), save_path_(get_save_path())
    {
        // Create a publisher for PoseWithCovarianceStamped messages on the "initialpose" topic with a queue size of 10
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

        // Declare a parameter for the timer period with a default value of 5 seconds
        this->declare_parameter<double>("timer_period", 5.0);
        double timer_period = this->get_parameter("timer_period").as_double();

        // Create a timer that triggers the load_and_publish_pose callback at the specified period
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period),
            std::bind(&InitialPosePublisher::load_and_publish_pose, this)
        );

        RCLCPP_INFO(this->get_logger(), "InitialPosePublisher node has been initialized.");
    }

private:
    /**
     * @brief Retrieves the path to the JSON file containing the last pose.
     * 
     * @return std::string The full path to the last_pose.json file.
     */
    std::string get_save_path() const {
        const char* home = std::getenv("HOME");
        if (!home) {
            RCLCPP_ERROR(this->get_logger(), "HOME environment variable is not set.");
            return "/tmp/last_pose.json"; // Fallback path
        }
        return std::string(home) + "/robot_data/pose/last_pose.json";
    }

    /**
     * @brief Loads the pose from the JSON file and publishes it as a PoseWithCovarianceStamped message.
     */
    void load_and_publish_pose()
    {
        // Check if the pose file exists
        if (!fs::exists(save_path_)) {
            RCLCPP_WARN(this->get_logger(), "Pose file does not exist: %s", save_path_.c_str());
            return;
        }

        std::ifstream file(save_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open pose file: %s", save_path_.c_str());
            return;
        }

        nlohmann::json pose_data;
        try {
            file >> pose_data;
        } catch (const nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
            return;
        }
        file.close();

        // Validate JSON structure
        if (!pose_data.contains("position") || !pose_data.contains("orientation")) {
            RCLCPP_ERROR(this->get_logger(), "JSON does not contain required fields.");
            return;
        }

        auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "map";

        // Assign position values        
        message.pose.pose.position.x = pose_data["position"]["x"];
        message.pose.pose.position.y = pose_data["position"]["y"];
        message.pose.pose.position.z = pose_data["position"]["z"];

        // Assign orientation values
        message.pose.pose.orientation.x = pose_data["orientation"]["x"];
        message.pose.pose.orientation.y = pose_data["orientation"]["y"];
        message.pose.pose.orientation.z = pose_data["orientation"]["z"];
        message.pose.pose.orientation.w = pose_data["orientation"]["w"];

        // Optionally, set covariance if available
        if (pose_data.contains("covariance")) {
            for (size_t i = 0; i < 36 && i < pose_data["covariance"].size(); ++i) {
                message.pose.covariance[i] = pose_data["covariance"][i].get<double>();
            }
        }

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Initial pose published.");
    } 


    // Publisher for PoseWithCovarianceStamped messages
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;

    // Timer to periodically publish the pose
    rclcpp::TimerBase::SharedPtr timer_;

    // Path to the JSON file containing the last pose
    std::string save_path_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the InitialPosePublisher node
    rclcpp::spin(std::make_shared<InitialPosePublisher>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
