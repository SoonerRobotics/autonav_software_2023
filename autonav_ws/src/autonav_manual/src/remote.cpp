#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "scr_core/node.h"

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

class JoyXboxNode : public SCR::Node
{
public:
	JoyXboxNode() : SCR::Node("autonav_manual_xbox") {}

	void configure() override
	{
		steamSubscription = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyXboxNode::onJoyReceived, this, _1));
		motorPublisher = this->create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 10);

		setDeviceState(SCR::DeviceState::READY);
	}

	void transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override
	{
		if (updated.state == SCR::SystemState::MANUAL && getDeviceState() == SCR::DeviceState::READY)
		{
			setDeviceState(SCR::DeviceState::OPERATING);
		}

		if (updated.state != SCR::SystemState::MANUAL && getDeviceState() == SCR::DeviceState::OPERATING)
		{
			setDeviceState(SCR::DeviceState::READY);
		}
	}

private:
	void onJoyReceived(const sensor_msgs::msg::Joy &msg) const
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
		motorPublisher->publish(package);
	}

	rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motorPublisher;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr steamSubscription;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JoyXboxNode>());
	rclcpp::shutdown();
	return 0;
}