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
#include "std_srvs/srv/trigger.hpp"

namespace fs = std::filesystem;

class InitialPosePublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the InitialPosePublisher node.
     * Initializes parameters, publisher, service, and sets up a timer for periodic publishing.
     */
    InitialPosePublisher() 
    : Node("initial_pose_publisher"), stop_publishing_(false), save_path_(get_save_path())
    {
        // Create a publisher for PoseWithCovarianceStamped messages on the "initialpose" topic with a queue size of 10
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        // Create a service to stop publishing
        stop_service_ = this->create_service<std_srvs::srv::Trigger>(
            "stop_initial_pose_publisher",
            std::bind(&InitialPosePublisher::stopCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Initialize a timer to publish pose every 1 second
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            [this]() { this->publish_pose(); }
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
        std::string path = std::string(home) + "/robot_data/pose/last_pose.json";
        fs::create_directories(fs::path(path).parent_path());
        return path;
    }

    /**
     * @brief Loads the pose from the JSON file and publishes it as a PoseWithCovarianceStamped message.
     * Continues publishing until a stop signal is received via the service.
     */
    void publish_pose()
    {
        if (stop_publishing_) {
            RCLCPP_INFO(this->get_logger(), "Publishing has been stopped.");
            publish_timer_->cancel();
            return;
        }

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

        // Create and fill the PoseWithCovarianceStamped message
        auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "map";
        fill_pose_data(message, pose_data);

        // Publish the message
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Initial pose published.");
    } 

    /**
     * @brief Fills the PoseWithCovarianceStamped message with the data from the JSON object.
     * 
     * @param message The PoseWithCovarianceStamped message to fill.
     * @param pose_data The JSON object containing the pose data.
     */
    void fill_pose_data(geometry_msgs::msg::PoseWithCovarianceStamped& message, const nlohmann::json& pose_data) const
    {
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
    }

    /**
     * @brief Callback function for the stop_initial_pose_publisher service.
     * 
     * @param request The service request.
     * @param response The service response.
     */
    void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        RCLCPP_INFO(this->get_logger(), "Stopping InitialPosePublisher.");
        stop_publishing_ = true;
        response->success = true;
        response->message = "InitialPosePublisher has been stopped.";
    }

    // Publisher for PoseWithCovarianceStamped messages
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;

    // Service to stop publishing
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

    // Timer for periodic pose publishing
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Flag to stop publishing
    bool stop_publishing_;

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
