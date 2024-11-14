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

class PoseSaver : public rclcpp::Node
{
public:
    PoseSaver() : Node("pose_saver"), save_path_(get_save_path())
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&PoseSaver::pose_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::minutes(1),
            std::bind(&PoseSaver::save_pose_to_file, this));
    }

private:
    std::string get_save_path() {
        auto home = std::getenv("HOME");
        return std::string(home) + "/robot_data/pose/last_pose.json";
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
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
            }}
        };
    }

    void save_pose_to_file() {
        std::ofstream file(save_path_);
        if (file.is_open()) {
            file << last_pose_.dump(4);
            file.close();
            RCLCPP_INFO(this->get_logger(), "Pose saved to %s", save_path_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open pose file.");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    nlohmann::json last_pose_;
    std::string save_path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSaver>());
    rclcpp::shutdown();
    return 0;
}
