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
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;

class SensorSyncNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the SensorSyncNode.
     * 
     * Initializes subscribers, synchronizer, publishers, and the transform broadcaster.
     */
    SensorSyncNode()
    : Node("sensor_sync_node"),
      x_(0.0),
      y_(0.0),
      theta_(0.0),
      wheel_base_(0.5),  // Wheel base in meters
      last_time_(this->now())
    {
        // Define QoS settings for the subscriptions and publications
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // Initialize message_filters subscribers for IMU, Odometry, LaserScan, and wheel velocities
        imu_sub_.subscribe(this, "/imu/data_raw", qos);
        odom_sub_.subscribe(this, "/odom", qos);
        scan_sub_.subscribe(this, "/scan", qos);
        left_wheel_sub_.subscribe(this, "/left_wheel/velocity", qos);
        right_wheel_sub_.subscribe(this, "/right_wheel/velocity", qos);

        // Set up ApproximateTimeSynchronizer with a policy that synchronizes all five topics
        sync_.reset(new Sync(MySyncPolicy(10), imu_sub_, odom_sub_, scan_sub_, left_wheel_sub_, right_wheel_sub_));
        sync_->registerCallback(std::bind(&SensorSyncNode::sync_callback, this, _1, _2, _3, _4, _5));

        // Initialize publishers for synchronized data to be used by Cartographer or other nodes
        carto_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/synchronized_imu", qos);
        carto_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/synchronized_odom", qos);
        carto_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/synchronized_scan", qos);

        // Initialize Transform Broadcaster for publishing odometry transforms
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "SensorSyncNode initialized and subscribers set up.");
    }

private:
    /**
     * @brief Callback function triggered when synchronized messages are received.
     * 
     * This function is called when all subscribed messages are received within the synchronization window.
     * It calculates odometry based on wheel velocities, publishes synchronized data, and broadcasts transforms.
     * 
     * @param imu_msg Pointer to the received IMU message.
     * @param odom_msg Pointer to the received Odometry message.
     * @param scan_msg Pointer to the received LaserScan message.
     * @param left_wheel_msg Pointer to the received left wheel velocity message.
     * @param right_wheel_msg Pointer to the received right wheel velocity message.
     */
    void sync_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
                      const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
                      const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
                      const geometry_msgs::msg::TwistStamped::ConstSharedPtr left_wheel_msg,
                      const geometry_msgs::msg::TwistStamped::ConstSharedPtr right_wheel_msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Synchronized callback triggered.");

        // Calculate odometry based on wheel velocities
        nav_msgs::msg::Odometry calculated_odom = calculate_odometry(left_wheel_msg, right_wheel_msg);

        // Publish the synchronized data to Cartographer
        carto_imu_pub_->publish(*imu_msg);
        carto_odom_pub_->publish(*odom_msg);
        carto_scan_pub_->publish(*scan_msg);

        // Broadcast the odometry transform
        broadcast_transform(calculated_odom);

        RCLCPP_INFO(this->get_logger(), "Synchronized IMU, Calculated Odometry, and LaserScan data published.");
    }

    /**
     * @brief Calculates odometry based on left and right wheel velocities.
     * 
     * This function computes the robot's position and orientation by integrating wheel velocities over time.
     * 
     * @param left_wheel_msg Pointer to the left wheel velocity message.
     * @param right_wheel_msg Pointer to the right wheel velocity message.
     * @return Calculated Odometry message.
     */
    nav_msgs::msg::Odometry calculate_odometry(const geometry_msgs::msg::TwistStamped::ConstSharedPtr left_wheel_msg,
                                              const geometry_msgs::msg::TwistStamped::ConstSharedPtr right_wheel_msg)
    {
        // Extract linear velocities from wheel messages
        double left_velocity = left_wheel_msg->twist.linear.x;
        double right_velocity = right_wheel_msg->twist.linear.x;

        // Compute average linear velocity and angular velocity
        double linear_velocity = (left_velocity + right_velocity) / 2.0;
        double angular_velocity = (right_velocity - left_velocity) / wheel_base_;

        // Get current time and compute time difference since last update
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Update the pose based on the velocities and time difference
        x_ += linear_velocity * cos(theta_) * dt;
        y_ += linear_velocity * sin(theta_) * dt;
        theta_ += angular_velocity * dt;

        // Normalize theta_ to stay within [-pi, pi]
        theta_ = normalize_angle(theta_);

        // Create and return the Odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Set pose
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        // Convert theta to quaternion for orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        // Set velocity
        odom.twist.twist.linear.x = linear_velocity;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = angular_velocity;

        return odom;
    }

    /**
     * @brief Broadcasts the odometry transform based on the odometry message.
     * 
     * This function publishes the transform between the "odom" frame and the "base_link" frame.
     * 
     * @param odom_msg Reference to the Odometry message.
     */
    void broadcast_transform(const nav_msgs::msg::Odometry& odom_msg)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = odom_msg.header.stamp;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";

        transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
        transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation = odom_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    /**
     * @brief Normalizes an angle to the range [-pi, pi].
     * 
     * This function ensures that the angle remains within the specified range to prevent overflow.
     * 
     * @param angle The angle in radians to normalize.
     * @return The normalized angle.
     */
    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // Message_filters subscribers for synchronized topics
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped, SensorSyncNode> left_wheel_sub_;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped, SensorSyncNode> right_wheel_sub_;

    // Define the synchronization policy with ApproximateTime for five topics
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu,
                                                            nav_msgs::msg::Odometry,
                                                            sensor_msgs::msg::LaserScan,
                                                            geometry_msgs::msg::TwistStamped,
                                                            geometry_msgs::msg::TwistStamped> MySyncPolicy;

    // Synchronizer for the five topics
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    // Publishers for synchronized data
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr carto_imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr carto_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr carto_scan_pub_;

    // Transform broadcaster for publishing odometry transforms
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Variables for odometry calculation
    double x_;         // X position in odom frame
    double y_;         // Y position in odom frame
    double theta_;     // Orientation in radians
    double wheel_base_; // Distance between wheels in meters
    rclcpp::Time last_time_; // Time of the last odometry update
};

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the SensorSyncNode
    auto node = std::make_shared<SensorSyncNode>();
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
