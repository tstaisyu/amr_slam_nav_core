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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>
#include <string>

class PosePublisherNode : public rclcpp::Node
{
public:
    PosePublisherNode() 
    : Node("pose_publisher_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("source_frame", "map");
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("pose_topic", "pose_estimate");
        this->declare_parameter<int>("publish_rate_hz", 10);
        this->declare_parameter<double>("covariance_x", 0.01);
        this->declare_parameter<double>("covariance_y", 0.01);
        this->declare_parameter<double>("covariance_yaw", 0.02);
        
        this->get_parameter("source_frame", source_frame_);
        this->get_parameter("target_frame", target_frame_);
        this->get_parameter("pose_topic", pose_topic_);
        this->get_parameter("publish_rate_hz", publish_rate_hz_);
        this->get_parameter("covariance_x", covariance_x_);
        this->get_parameter("covariance_y", covariance_y_);
        this->get_parameter("covariance_yaw", covariance_yaw_);

        RCLCPP_INFO(this->get_logger(), "Source Frame: %s", source_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target Frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Pose Topic: %s", pose_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish Rate: %d Hz", publish_rate_hz_);
        RCLCPP_INFO(this->get_logger(), "Covariance X: %.4f", covariance_x_);
        RCLCPP_INFO(this->get_logger(), "Covariance Y: %.4f", covariance_y_);
        RCLCPP_INFO(this->get_logger(), "Covariance Yaw: %.4f", covariance_yaw_);

        // Initialize publisher
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_estimate", 10);
        
        // Calculate publish period based on rate
        auto publish_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_hz_));
        
        // Initialize timer
        timer_ = this->create_wall_timer(
            publish_period, 
            std::bind(&PosePublisherNode::publishPose, this));
        
        RCLCPP_INFO(this->get_logger(), "PosePublisherNode has been initialized.");
    }

private:
    void publishPose()
    {
        try {
            // Lookup the latest transform from source_frame to target_frame
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped = tf_buffer_.lookupTransform(
                source_frame_, target_frame_, tf2::TimePointZero);
            
            // Convert Transform to PoseWithCovarianceStamped message
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = this->get_clock()->now();
            pose_msg.header.frame_id = source_frame_;
            
            pose_msg.pose.pose.position.x = transformStamped.transform.translation.x;
            pose_msg.pose.pose.position.y = transformStamped.transform.translation.y;
            pose_msg.pose.pose.position.z = transformStamped.transform.translation.z;
            pose_msg.pose.pose.orientation = transformStamped.transform.rotation;

            // Initialize covariance matrix to zero
            for (int i = 0; i < 36; i++) {
                pose_msg.pose.covariance[i] = 0.0;
            }
            // Set specific covariance values
            pose_msg.pose.covariance[0] = covariance_x_;  // x variance
            pose_msg.pose.covariance[7] = covariance_y_;  // y variance
            pose_msg.pose.covariance[35] = covariance_yaw_; // yaw variance

            // Publish the pose message
            pose_publisher_->publish(pose_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published pose message.");
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", 
                         source_frame_.c_str(), target_frame_.c_str(), ex.what());
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(this->get_logger(), "An unexpected error occurred: %s", ex.what());
        }
    }

    // Timer for publishing pose
    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher for PoseWithCovarianceStamped
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    std::string source_frame_;
    std::string target_frame_;
    std::string pose_topic_;
    int publish_rate_hz_;
    double covariance_x_;
    double covariance_y_;
    double covariance_yaw_;};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PosePublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
