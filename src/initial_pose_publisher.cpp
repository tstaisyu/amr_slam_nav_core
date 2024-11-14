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

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher() : Node("initial_pose_publisher"), save_path_(get_save_path())
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&InitialPosePublisher::load_and_publish_pose, this));
    }

private:
    std::string get_save_path() {
        auto home = std::getenv("HOME");
        return std::string(home) + "/robot_data/pose/last_pose.json";
    }

    void load_and_publish_pose()
    {
        std::ifstream file(save_path_);
        if (file.is_open()) {
            nlohmann::json pose_data;
            file >> pose_data;
            file.close();

            auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "map";
            message.pose.pose.position.x = pose_data["position"]["x"];
            message.pose.pose.position.y = pose_data["position"]["y"];
            message.pose.pose.position.z = pose_data["position"]["z"];
            message.pose.pose.orientation.x = pose_data["orientation"]["x"];
            message.pose.pose.orientation.y = pose_data["orientation"]["y"];
            message.pose.pose.orientation.z = pose_data["orientation"]["z"];
            message.pose.pose.orientation.w = pose_data["orientation"]["w"];

            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Initial pose published.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open pose file.");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string save_path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}
