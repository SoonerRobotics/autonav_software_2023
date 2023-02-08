#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autonav_msgs/msg/motor_input.hpp"

using std::placeholders::_1;

class JoySubscriber : public rclcpp::Node
{
public:
  JoySubscriber() : Node("autonav_manual_remote")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoySubscriber::on_joy_received, this, _1));

    motor_publisher = this->create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 10);
  }

private:
  void on_joy_received(const sensor_msgs::msg::Joy &msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received Joy: %f | %f", msg.axes[1], msg.axes[3]);

    autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
    package.left_motor = msg.axes[1];
    package.right_motor = msg.axes[3];
    motor_publisher->publish(package);
  }

  rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motor_publisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoySubscriber>());
  rclcpp::shutdown();
  return 0;
}