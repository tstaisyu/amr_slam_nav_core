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
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class M5ConnectInitializer : public rclcpp::Node {
public:
    M5ConnectInitializer() : Node("m5_connect_initializer") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(300),
            std::bind(&M5ConnectInitializer::publish_zero_velocity, this)
        );
        start_time_ = this->now();
    }

private:
    void publish_zero_velocity() {
        auto current_time = this->now();
        if ((current_time - start_time_).seconds() < 15.0) {
            auto message = geometry_msgs::msg::Twist();
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);
        } else {
            timer_->cancel();
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<M5ConnectInitializer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
