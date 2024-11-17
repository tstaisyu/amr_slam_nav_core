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

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class WiFiConfigPublisher : public rclcpp::Node
{
public:
    WiFiConfigPublisher()
    : Node("wifi_config_publisher")
    {
        // パラメータの宣言とデフォルト値の設定
        this->declare_parameter<std::string>("ssid", "default_ssid");
        this->declare_parameter<std::string>("password", "default_password");

        // パラメータの取得
        this->get_parameter("ssid", ssid_);
        this->get_parameter("password", password_);

        // QoS設定の作成（micro-ROSのデフォルトに合わせて）
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        // パブリッシャの作成
        wifi_config_publisher_ = this->create_publisher<std_msgs::msg::String>("wifi_config", qos);

        // タイマーの作成（メッセージを一度だけ送信）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&WiFiConfigPublisher::publish_wifi_config, this)
        );
    }

private:
    void publish_wifi_config()
    {
        // メッセージのフォーマット
        std::string message_data = "SSID:" + ssid_ + ";PASSWORD:" + password_;
        auto message = std_msgs::msg::String();
        message.data = message_data;

        RCLCPP_INFO(this->get_logger(), "Publishing WiFi config: '%s'", message.data.c_str());

        // メッセージの送信
        wifi_config_publisher_->publish(message);

        // 一度だけ送信する場合はタイマーをキャンセル
        timer_->cancel();
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wifi_config_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string ssid_;
    std::string password_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WiFiConfigPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
