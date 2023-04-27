#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/steam_input.hpp"
#include "scr_core/device_state.h"
#include "std_msgs/msg/float32.hpp"
#include "scr_core/node.h"

using std::placeholders::_1;

long lastMessageTime = 0;
float lastForwardSpeed = 0;
float lastTurnSpeed = 0;

float clamp(float value, float min, float max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

enum Registers
{
	STEERING_DEADZONE = 1,
	THROTTLE_DEADZONE = 2,
	FORWARD_SPEED = 3,
	TURN_SPEED = 4
};

class SteamJoyNode : public SCR::Node
{
public:
	SteamJoyNode() : SCR::Node("autonav_manual_steamcontroller") {}

	void configure() override
	{
		steamSubscription = create_subscription<autonav_msgs::msg::SteamInput>("/autonav/joy/steam", 20, std::bind(&SteamJoyNode::onSteamDataReceived, this, _1));
		motorPublisher = create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 20);

		config.set(Registers::STEERING_DEADZONE, 0.04f);
		config.set(Registers::THROTTLE_DEADZONE, 0.04f);
		config.set(Registers::FORWARD_SPEED, 1.5f);
		config.set(Registers::TURN_SPEED, 1.0f);

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

	void onSpeedReceived(const std_msgs::msg::Float32::SharedPtr msg)
	{
		speed = msg->data;
	}

	void onSteamDataReceived(const autonav_msgs::msg::SteamInput &msg)
	{
		lastMessageTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		if (getSystemState().state != SCR::SystemState::MANUAL || getDeviceState() != SCR::DeviceState::OPERATING)
		{
			return;
		}

		float throttle = 0;
		float steering = 0;
		const float throttleDeadzone = config.get<float>(Registers::THROTTLE_DEADZONE);
		const float steeringDeadzone = config.get<float>(Registers::STEERING_DEADZONE);

		if (abs(msg.ltrig) > throttleDeadzone || abs(msg.rtrig) > throttleDeadzone)
		{
			throttle = msg.rtrig * config.get<float>(Registers::FORWARD_SPEED);
			throttle = throttle - msg.ltrig * config.get<float>(Registers::FORWARD_SPEED);
		}

		if (abs(msg.lpad_x) > steeringDeadzone)
		{
			steering = msg.lpad_x * config.get<float>(Registers::TURN_SPEED);
		}

		autonav_msgs::msg::MotorInput input;
		input.forward_velocity = clamp(throttle, -2.2f, 2.2f);
		input.angular_velocity = clamp(steering, -3.0f, 3.0f);
		motorPublisher->publish(input);
	}

	rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motorPublisher;
	rclcpp::Subscription<autonav_msgs::msg::SteamInput>::SharedPtr steamSubscription;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speedSubscription;
	rclcpp::TimerBase::SharedPtr heartbeatTimer;
	float speed = 0.6f;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SteamJoyNode>());
	rclcpp::shutdown();
	return 0;
}
