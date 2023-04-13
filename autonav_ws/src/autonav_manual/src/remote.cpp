#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_libs/common.h"

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

class LoggingNode : public Autonav::ROS::AutoNode
{
public:
	LoggingNode() : AutoNode("autonav_manual_xbox") {}

	void setup() override
	{
		m_steamSubscription = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&LoggingNode::on_joy_received, this, _1));
		m_motorPublisher = this->create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 10);
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


		package.forward_velocity = throttle;
		package.angular_velocity = steering;
		m_motorPublisher->publish(package);
	}

	rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr m_motorPublisher;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_steamSubscription;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LoggingNode>());
	rclcpp::shutdown();
	return 0;
}