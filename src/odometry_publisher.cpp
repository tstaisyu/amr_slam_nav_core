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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>

class OdometryPublisher : public rclcpp::Node {
public:
    OdometryPublisher() 
    : Node("odometry_publisher"), update_rate_(50.0), wheel_base_(0.202),
      alpha_(0.2), left_filtered_velocity_(0.0), right_filtered_velocity_(0.0) {

        // QoS settings
        rclcpp::QoS custom_qos_profile(10);
        custom_qos_profile.best_effort();

        // Subscribing to left and right wheel velocity topics
        left_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "left_wheel/velocity", custom_qos_profile, 
            std::bind(&OdometryPublisher::left_wheel_callback, this, std::placeholders::_1));
        right_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "right_wheel/velocity", custom_qos_profile, 
            std::bind(&OdometryPublisher::right_wheel_callback, this, std::placeholders::_1));

        // Publisher for odometry data
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/odom_encoder", 10);

        // Initialize robot state
        x_ = 0.0; y_ = 0.0; theta_ = 0.0;
        last_time_ = this->get_clock()->now();    

        // Timer for periodic odometry update
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / update_rate_),
            std::bind(&OdometryPublisher::update_odometry, this));    

        RCLCPP_INFO(this->get_logger(), "Odometry publisher has started.");
    }

private:
    // Handle left wheel velocity messages
    void left_wheel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        double measured_velocity = msg->twist.linear.x;
        left_filtered_velocity_ = alpha_ * measured_velocity + (1 - alpha_) * left_filtered_velocity_;
        detect_and_handle_slip();
    }

    // Handle right wheel velocity messages
    void right_wheel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        double measured_velocity = msg->twist.linear.x;
        right_filtered_velocity_ = alpha_ * measured_velocity + (1 - alpha_) * right_filtered_velocity_;
        detect_and_handle_slip();
    }

    // Detect wheel slip based on velocity difference and take corrective action
    void detect_and_handle_slip(nav_msgs::msg::Odometry::SharedPtr odom_msg = nullptr) {
        double wheel_speed_difference = std::abs(left_filtered_velocity_ - right_filtered_velocity_);
        if (wheel_speed_difference > slip_threshold_) {
            if (!slip_detected_) {
                RCLCPP_WARN(this->get_logger(), "Wheel slip detected! Speed difference: %f", wheel_speed_difference);
                slip_detected_ = true;
                adjust_odometry_covariance(true, odom_msg);
            }
        } else {
            if (slip_detected_) {
                RCLCPP_INFO(this->get_logger(), "Wheel slip resolved.");
                slip_detected_ = false;
                adjust_odometry_covariance(false, odom_msg);
            }
        }
    }

    // オドメトリの共分散を調整するメソッド
    void adjust_odometry_covariance(bool is_slipping, nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        // スリップ時は共分散を大きく設定（信頼性を低減）
        if (is_slipping) {
            odom_msg->pose.covariance[0] = 1.0;  // Position x
            odom_msg->pose.covariance[7] = 1.0;  // Position y
            odom_msg->pose.covariance[35] = 1.0; // Orientation yaw
            odom_msg->twist.covariance[0] = 1.0; // Velocity x
            odom_msg->twist.covariance[35] = 1.0; // Angular velocity z
        } else {
            // 通常時の共分散に戻す
            odom_msg->pose.covariance[0] = 0.01;  // Position x
            odom_msg->pose.covariance[7] = 0.01;  // Position y
            odom_msg->pose.covariance[35] = 0.05; // Orientation yaw
            odom_msg->twist.covariance[0] = 0.01; // Velocity x
            odom_msg->twist.covariance[35] = 0.05; // Angular velocity z
        }
    }

    void update_odometry() {
        if (slip_detected_) {
            RCLCPP_WARN(this->get_logger(), "Skipping odometry update due to slip.");
            return; // Skip this update if slip is detecteds
        }

        // Get the current time for this update cycle
        auto current_time = this->get_clock()->now();
        // Calculate the time elapsed since the last update
        double dt = (current_time - last_time_).seconds();

        RCLCPP_DEBUG(this->get_logger(), "Time delta: %f", dt);

        // Check for valid time delta: It should be positive and not unreasonably large
        if (dt <= 0.0 || dt > 0.1) {
            RCLCPP_WARN(this->get_logger(), "Invalid time delta: %f", dt);
            return; // Skip this update if the time delta is invalid
        }

        // Update the last_time_ to current time for the next cycle
        last_time_ = current_time;

        // Compute linear and angular velocities from the wheel velocities
        double linear_velocity = (left_filtered_velocity_ + right_filtered_velocity_) / 2.0;
        double angular_velocity = (right_filtered_velocity_ - left_filtered_velocity_) / wheel_base_;

        // Calculate the change in position and orientation
        double delta_x = linear_velocity * cos(theta_) * dt; // Change in x based on current orientation and speed
        double delta_y = linear_velocity * sin(theta_) * dt; // Change in y based on current orientation and speed
        theta_ += angular_velocity * dt; // Update the orientation based on angular velocity

        // Normalize the orientation to the range [-pi, pi]
        theta_ = std::fmod(theta_, 2.0 * M_PI);
        if (theta_ < -M_PI) theta_ += 2.0 * M_PI;
        if (theta_ > M_PI) theta_ -= 2.0 * M_PI;

        x_ += delta_x; // Update the x position
        y_ += delta_y; // Update the y position

        RCLCPP_DEBUG(this->get_logger(), "Updated odometry: x=%f, y=%f, theta=%f", x_, y_, theta_);

        // Finally, publish the updated odometry to the rest of the system
        publish_odometry(linear_velocity, angular_velocity);
    }

    // Publish the computed odometry
    void publish_odometry(double linear_velocity, double angular_velocity) {
        // Create and publish odometry message
        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = this->get_clock()->now();
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_link";
        populate_odometry_message(linear_velocity, angular_velocity, odom_msg);
    }

    // Fill the odometry message with current state
    void populate_odometry_message(double linear_velocity, double angular_velocity, nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        // Set position and orientation in odometry message
        odom_msg->pose.pose.position.x = x_;
        odom_msg->pose.pose.position.y = y_;
        odom_msg->pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg->pose.pose.orientation = tf2::toMsg(q);

        // Set linear and angular velocities in odometry message
        odom_msg->twist.twist.linear.x = linear_velocity;
        odom_msg->twist.twist.angular.z = angular_velocity;

        // Set covariance matrices, assuming some standard values
        std::fill(std::begin(odom_msg->pose.covariance), std::end(odom_msg->pose.covariance), 0.0);
        std::fill(std::begin(odom_msg->twist.covariance), std::end(odom_msg->twist.covariance), 0.0);
        odom_msg->pose.covariance[0] = 0.01;  // Position x
        odom_msg->pose.covariance[7] = 0.01;  // Position y
        odom_msg->pose.covariance[35] = 0.05;  // Orientation yaw
        odom_msg->twist.covariance[0] = 0.01; // Velocity x
        odom_msg->twist.covariance[35] = 0.05; // Angular velocity z

        odom_publisher_->publish(*odom_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published odometry message.");
    }

    // Member variables
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr left_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr right_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double alpha_; // Exponential smoothing coefficient
    double left_filtered_velocity_;
    double right_filtered_velocity_;

    double x_{0.0}, y_{0.0}, theta_{0.0};  // Robot's position and orientation
    rclcpp::Time last_time_;
    const double update_rate_;  // Update rate in Hz
    const double wheel_base_;  // Distance between the wheels

    bool slip_detected_ = false;
    const double slip_threshold_ = 0.5;  // Threshold for wheel slip detection
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
