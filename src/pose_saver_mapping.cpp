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
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/TransformStamped.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <cstdlib>
#include <filesystem>

namespace fs = std::filesystem;

class PoseSaver : public rclcpp::Node
{
public:
    PoseSaver() : Node("pose_saver_mapping"), tf_buffer_(), tf_listener_(tf_buffer_)
    {
        save_timer_ = this->create_wall_timer(
            std::chrono::seconds(10), // デモ用に10秒ごとにチェック
            std::bind(&PoseSaver::save_pose, this));
    }

private:
    std::string get_save_path() {
        const char* home = std::getenv("HOME");
        std::string path = std::string(home ? home : "/tmp") + "/robot_data/pose/last_pose.json";
        fs::create_directories(fs::path(path).parent_path());
        return path;
    }

    void save_pose()
    {
        try {
            auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
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
                }}
            };

            std::ofstream file(save_path_);
            file << pose_data.dump(4);
            file.close();
            RCLCPP_INFO(this->get_logger(), "Pose saved successfully.");
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr save_timer_;
    std::string save_path_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSaver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
