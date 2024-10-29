#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class RawIMUSubscriber : public rclcpp::Node
{
public:
    RawIMUSubscriber(): Node("raw_imu_subscriber") {
        
        // QoSをBEST_EFFORTに設定
        auto subscriber_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        subscriber_qos.best_effort();

        // QoSをRELIABLEに設定
        auto publisher_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        publisher_qos.reliable();

        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw",
            subscriber_qos,
            std::bind(&RawIMUSubscriber::imu_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "imu/data_filtered",
            publisher_qos
        );
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto filtered_msg = filter_data(msg);
        publisher_->publish(*filtered_msg);
    }

    sensor_msgs::msg::Imu::SharedPtr filter_data(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // ここにフィルタリングロジックを実装
        // 例えば、ノイズリダクション、バイアス補正など

        return msg; // 変更後のメッセージを返す
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RawIMUSubscriber>());
    rclcpp::shutdown();
    return 0;
}