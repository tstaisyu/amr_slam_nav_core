#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class OdometryPublisher : public rclcpp::Node {
public:
    OdometryPublisher() : Node("odometry_publisher") {
        left_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "left_vel", 10, std::bind(&OdometryPublisher::left_wheel_callback, this, std::placeholders::_1));
        right_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "right_vel", 10, std::bind(&OdometryPublisher::right_wheel_callback, this, std::placeholders::_1));
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize position and orientation
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
    }

private:
    void left_wheel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        left_velocity_ = msg->twist.linear.x;
    }

    void right_wheel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        right_velocity_ = msg->twist.linear.x;
        update_odometry();
    }

    void update_odometry() {
        double dt = this->get_clock()->now().seconds() - last_time_;
        last_time_ = this->get_clock()->now().seconds();

        // Compute odometry here based on left_velocity_ and right_velocity_
        double linear_velocity = (left_velocity_ + right_velocity_) / 2;
        double angular_velocity = (right_velocity_ - left_velocity_) / wheel_base_;

        // Update robot position and orientation
        double delta_x = linear_velocity * cos(theta_) * dt;
        double delta_y = linear_velocity * sin(theta_) * dt;
        theta_ += angular_velocity * dt;
        x_ += delta_x;
        y_ += delta_y;

        // Publish odometry message
        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = this->get_clock()->now();
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_link";

        odom_msg->pose.pose.position.x = x_;
        odom_msg->pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg->pose.pose.orientation.x = q.x();
        odom_msg->pose.pose.orientation.y = q.y();
        odom_msg->pose.pose.orientation.z = q.z();
        odom_msg->pose.pose.orientation.w = q.w();

        odom_publisher_->publish(*odom_msg);

        // Send transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr left_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr right_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double left_velocity_{0.0};
    double right_velocity_{0.0};
    double x_, y_, theta_;
    double last_time_{0.0};
    const double wheel_base_ = 0.5;  // Assume some wheel base
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}