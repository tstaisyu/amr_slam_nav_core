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
#include <cstdlib>
#include <string>
#include <filesystem>

namespace fs = std::filesystem;

/**
 * @brief Node that subscribes to the "amcl_pose" topic and periodically saves the latest pose to a JSON file.
 */
class PoseSaverNav : public rclcpp::Node
{
public:
    PoseSaverNav() : Node("pose_saver_nav"), save_path_(get_save_path())
    {
        // Initialize subscription to "amcl_pose" topic with QoS depth 10
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 
            10, 
            std::bind(&PoseSaverNav::pose_callback, this, std::placeholders::_1));

        // Initialize timer to trigger every 60 seconds to save pose
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PoseSaverNav::save_pose_to_file, this));

        RCLCPP_INFO(this->get_logger(), "PoseSaver node initialized. Saving poses to %s every 60 seconds.", save_path_.c_str());
    }

    ~PoseSaverNav()
    {
        // Save the last pose upon destruction (node shutdown)
        if (!last_pose_.empty()) {
            save_pose_to_file();
        }
    }
    
private:
    /**
     * @brief Retrieves the path to save the last pose JSON file.
     * 
     * @return std::string The full path to the last_pose.json file.
     */
    std::string get_save_path() {
        const char* home = std::getenv("HOME");
        if (!home) {
            RCLCPP_ERROR(this->get_logger(), "HOME environment variable is not set. Using default save path /tmp/last_pose.json");
            return "/tmp/last_pose.json"; // Fallback path
        }

        std::string directory = std::string(home) + "/robot_data/pose";
        std::string file_path = directory + "/last_pose.json";

        // Ensure the directory exists
        if (!fs::exists(directory)) {
            try {
                fs::create_directories(directory);
                RCLCPP_INFO(this->get_logger(), "Created directory: %s", directory.c_str());
            } catch (const fs::filesystem_error& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s", directory.c_str(), e.what());
                // Fallback to /tmp
                return "/tmp/last_pose.json";
            }
        }

        return file_path;
    }

    /**
     * @brief Callback function for the "amcl_pose" topic.
     * 
     * @param msg The received PoseWithCovarianceStamped message.
     */
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Update the last_pose_ with the received message data
        last_pose_ = {
            {"position", {
                {"x", msg->pose.pose.position.x},
                {"y", msg->pose.pose.position.y},
                {"z", msg->pose.pose.position.z}
            }},
            {"orientation", {
                {"x", msg->pose.pose.orientation.x},
                {"y", msg->pose.pose.orientation.y},
                {"z", msg->pose.pose.orientation.z},
                {"w", msg->pose.pose.orientation.w}
            }},
            {"covariance", msg->pose.covariance} // Include covariance if needed
        };

        RCLCPP_DEBUG(this->get_logger(), "Pose updated from amcl_pose.");
    }

    /**
     * @brief Saves the last received pose to the JSON file.
     */
    void save_pose_to_file() {
        if (last_pose_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No pose data available to save.");
            return;
        }

        std::ofstream file(save_path_);
        if (file.is_open()) {
            try {
                file << last_pose_.dump(4); // Pretty-print with 4-space indentation
                file.close();
                RCLCPP_INFO(this->get_logger(), "Pose saved to %s", save_path_.c_str());
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write pose data to file: %s", e.what());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open pose file: %s", save_path_.c_str());
        }
    }

    // Subscriber to the "amcl_pose" topic
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;

    // Timer to periodically save the pose to file
    rclcpp::TimerBase::SharedPtr timer_;

    // JSON object to store the last received pose
    nlohmann::json last_pose_;

    // Path to save the last pose JSON file
    std::string save_path_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the PoseSaver node
    rclcpp::spin(std::make_shared<PoseSaverNav>());

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
