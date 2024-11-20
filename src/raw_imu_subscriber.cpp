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
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <string>

class RawIMUSubscriber : public rclcpp::Node
{
public:
    RawIMUSubscriber()
    : Node("raw_imu_subscriber"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("raw_imu_topic", "imu/data_raw");
        this->declare_parameter<std::string>("filtered_imu_topic", "imu/data_qos");
        this->declare_parameter<std::string>("source_frame", "map");
        this->declare_parameter<std::string>("target_frame", "imu_link");
        this->declare_parameter<int>("subscriber_queue_size", 10);
        this->declare_parameter<int>("publisher_queue_size", 10);
        this->declare_parameter<double>("offset_x", -0.045);  // meters
        this->declare_parameter<double>("offset_y", 0.105);   // meters
        this->declare_parameter<double>("offset_z", 0.10);    // meters
        this->declare_parameter<double>("yaw_offset", 0.0);   // radians

        // Retrieve parameters
        this->get_parameter("raw_imu_topic", raw_imu_topic_);
        this->get_parameter("filtered_imu_topic", filtered_imu_topic_);
        this->get_parameter("source_frame", source_frame_);
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("subscriber_queue_size", subscriber_queue_size_);
        this->get_parameter("publisher_queue_size", publisher_queue_size_);
        this->get_parameter("offset_x", offset_x_);
        this->get_parameter("offset_y", offset_y_);
        this->get_parameter("offset_z", offset_z_);
        this->get_parameter("yaw_offset", yaw_offset_);

        // Log parameter values
        RCLCPP_INFO(this->get_logger(), "Raw IMU Topic: %s", raw_imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Filtered IMU Topic: %s", filtered_imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Source Frame: %s", source_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target Frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscriber Queue Size: %d", subscriber_queue_size_);
        RCLCPP_INFO(this->get_logger(), "Publisher Queue Size: %d", publisher_queue_size_);
        RCLCPP_INFO(this->get_logger(), "IMU Offset - X: %.3f, Y: %.3f, Z: %.3f meters", offset_x_, offset_y_, offset_z_);
        RCLCPP_INFO(this->get_logger(), "IMU Yaw Offset: %.3f radians", yaw_offset_);

        // Set QoS to best effort for subscription to raw IMU data
        auto subscriber_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        subscriber_qos.best_effort();

        // Set QoS to reliable for publishing filtered IMU data
        auto publisher_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        publisher_qos.reliable();

        // Create subscription to raw IMU data
        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw",
            subscriber_qos,
            std::bind(&RawIMUSubscriber::imu_callback, this, std::placeholders::_1)
        );

        // Create publisher for filtered IMU data
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_qos",
            publisher_qos
        );
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Copy the incoming IMU data
        auto corrected_msg = *msg;

        // Implement filtering logic here
        correct_imu_data(corrected_msg);

        // Change the frame ID to 'imu_link'
        corrected_msg.header.frame_id = "imu_link";

        // Publish the filtered IMU data
        publisher_->publish(corrected_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published corrected IMU data.");
    }

    void correct_imu_data(sensor_msgs::msg::Imu &msg) {
        // Create offset position vector
        tf2::Vector3 r(offset_x_, offset_y_, offset_z_);

        // Create yaw offset quaternion
        tf2::Quaternion q_offset;
        q_offset.setRPY(0.0, 0.0, yaw_offset_);

        // Get the current orientation from the message
        tf2::Quaternion q_msg;
        tf2::fromMsg(msg.orientation, q_msg);

        // Apply orientation correction
        tf2::Quaternion q_corrected = q_offset * q_msg;
        q_corrected.normalize();
        msg.orientation = tf2::toMsg(q_corrected);

        // Get angular velocity from the message
        tf2::Vector3 omega(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);

        // Apply angular velocity correction
        tf2::Vector3 omega_corrected = tf2::quatRotate(q_offset, omega);
        msg.angular_velocity.x = omega_corrected.x();
        msg.angular_velocity.y = omega_corrected.y();
        msg.angular_velocity.z = omega_corrected.z();

        // Get linear acceleration from the message
        tf2::Vector3 acc(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);

        // Calculate centripetal acceleration
        tf2::Vector3 centripetal_acc = omega_corrected.cross(omega_corrected.cross(r));

        // Apply linear acceleration correction
        tf2::Vector3 acc_corrected = acc - centripetal_acc;
        acc_corrected = tf2::quatRotate(q_offset, acc_corrected);

        msg.linear_acceleration.x = acc_corrected.x();
        msg.linear_acceleration.y = acc_corrected.y();
        msg.linear_acceleration.z = acc_corrected.z();
    }

    // Subscriber to raw IMU data
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;

    // Publisher for filtered IMU data
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    std::string raw_imu_topic_;
    std::string filtered_imu_topic_;
    std::string source_frame_;
    std::string target_frame_;
    int subscriber_queue_size_;
    int publisher_queue_size_;
    double offset_x_;
    double offset_y_;
    double offset_z_;
    double yaw_offset_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RawIMUSubscriber>());
    rclcpp::shutdown();
    return 0;
}
