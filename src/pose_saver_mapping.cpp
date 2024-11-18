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
#include <fstream>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;

class PoseSaverMapping : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the PoseSaver node.
     * Initializes the TF buffer and listener, determines the save path, and sets up a timer.
     */
    PoseSaverMapping() 
    : Node("pose_saver_mapping"), 
      tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
      tf_listener_(tf_buffer_)
    {
        // Declare a parameter for the save interval in seconds
        this->declare_parameter<int>("save_interval_seconds", 1);
        int save_interval = this->get_parameter("save_interval_seconds").as_int();

        save_path_ = determine_save_path();
        initialize_timer(save_interval);

        RCLCPP_INFO(this->get_logger(), "PoseSaver node initialized.");
    }

private:
    /**
     * @brief Determines the file path where the pose will be saved.
     * Constructs the path based on the HOME environment variable and ensures the directory exists.
     *
     * @return std::string The full path to the JSON file where the pose will be saved.
     */
    std::string determine_save_path() {
        const char* home = std::getenv("HOME");
        std::string path = std::string(home ? home : "/tmp") + "/robot_data/pose/last_pose.json";
        fs::create_directories(fs::path(path).parent_path());
        return path;
    }

    /**
     * @brief Initializes a timer that triggers the save_pose method at regular intervals.
     * Currently set to trigger every 10 seconds for demonstration purposes.
     */
    void initialize_timer(int interval_seconds)
    {
        save_timer_ = this->create_wall_timer(
            std::chrono::seconds(interval_seconds), // Check every 10 seconds for demonstration
            std::bind(&PoseSaverMapping::save_pose, this)
        );
        RCLCPP_INFO(this->get_logger(), "Save timer initialized to trigger every 10 seconds.");
    }

    /**
     * @brief Retrieves the latest transform from the TF buffer and saves the pose to a JSON file.
     * Handles exceptions related to transform lookups and file I/O operations.
     */
    void save_pose()
    {
        try {
            // Lookup the latest transform from 'map' to 'base_link'
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero
            );

            // Construct JSON data with position, orientation, and covariance
            nlohmann::json pose_data = {
                {"position", {
                    {"x", transform.transform.translation.x},
                    {"y", transform.transform.translation.y},
                    {"z", transform.transform.translation.z}
                }},
                {"orientation", {
                    {"x", transform.transform.rotation.x},
                    {"y", transform.transform.rotation.y},
                    {"z", transform.transform.rotation.z},
                    {"w", transform.transform.rotation.w}
                }},
                {"covariance", {
                    // 36-element covariance matrix (example: identity matrix)
                    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0
                }}
            };

            // Open the file in write mode
            std::ofstream file(save_path_);
            if (file.is_open()) {
                file << pose_data.dump(4); // Write JSON with 4-space indentation
                file.close();
                RCLCPP_INFO(this->get_logger(), "Pose saved successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open pose file: %s", save_path_.c_str());
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
        } catch (const std::exception &ex) {
            RCLCPP_ERROR(this->get_logger(), "An unexpected error occurred: %s", ex.what());
        }
    }

    // TF buffer and listener for transform data
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Timer for periodic pose saving
    rclcpp::TimerBase::SharedPtr save_timer_;

    // Path to save the pose JSON file
    std::string save_path_;
};

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the PoseSaver node
    auto node = std::make_shared<PoseSaverMapping>();
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
