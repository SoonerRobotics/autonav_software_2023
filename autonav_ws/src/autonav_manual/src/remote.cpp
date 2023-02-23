#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autonav_msgs/msg/motor_input.hpp"

#define MAX_SPEED 1.4

using std::placeholders::_1;

float clamp(float value, float min, float max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

class SteamNode : public rclcpp::Node
{
public:
	SteamNode() : Node("autonav_manual_remote")
	{
		subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
			"/joy", 10, std::bind(&SteamNode::on_joy_received, this, _1));

		motor_publisher = this->create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 10);
	}

private:
	void on_joy_received(const sensor_msgs::msg::Joy &msg) const
	{
		float throttle = 0;
		float steering = 0;

		autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
		if (abs(msg.axes[5]) > 0.05 || abs(msg.axes[2]) > 0.05)
		{
			throttle = (1 - msg.axes[5]) * MAX_SPEED * 0.8;
			throttle = throttle - (1 - msg.axes[2]) * MAX_SPEED * 0.8;
		}

		if (abs(msg.axes[0]) > 0.15)
		{
			steering = msg.axes[0] * MAX_SPEED; 
		}


		package.left_motor = clamp(throttle - steering * 0.6, -MAX_SPEED, MAX_SPEED);
		package.right_motor = clamp(throttle + steering * 0.6, -MAX_SPEED, MAX_SPEED);
		motor_publisher->publish(package);
	}

	rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motor_publisher;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SteamNode>());
	rclcpp::shutdown();
	return 0;
}