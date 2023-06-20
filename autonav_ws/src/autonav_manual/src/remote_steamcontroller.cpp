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

namespace Registers
{
	const std::string STEERING_DEADZONE = "steering_deadzone";
	const std::string THROTTLE_DEADZONE = "throttle_deadzone";
	const std::string FORWARD_SPEED = "forward_speed";
	const std::string TURN_SPEED = "turn_speed";
	const std::string MAX_TURN_SPEED = "max_turn_speed";
	const std::string MAX_FORWARD_SPEED = "max_forward_speed";
};

class SteamJoyNode : public SCR::Node
{
public:
	SteamJoyNode() : SCR::Node("autonav_manual_steamcontroller") {}

	void configure() override
	{
		steamSubscription = create_subscription<autonav_msgs::msg::SteamInput>("/autonav/joy/steam", 20, std::bind(&SteamJoyNode::onSteamDataReceived, this, _1));
		motorPublisher = create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 20);

		config.set(Registers::STEERING_DEADZONE, 0.03f);
		config.set(Registers::THROTTLE_DEADZONE, 0.03f);
		config.set(Registers::FORWARD_SPEED, 1.8f);
		config.set(Registers::TURN_SPEED, 1.0f);
		config.set(Registers::MAX_TURN_SPEED, 3.14159265f);
		config.set(Registers::MAX_FORWARD_SPEED, 2.2f);

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
			throttle = msg.rtrig;
			throttle = throttle - msg.ltrig;
		}

		if (abs(msg.lpad_x) > steeringDeadzone)
		{
			steering = msg.lpad_x;
		}

		autonav_msgs::msg::MotorInput input;
		input.forward_velocity = clamp(throttle * config.get<float>(Registers::FORWARD_SPEED), -config.get<float>(Registers::MAX_FORWARD_SPEED), config.get<float>(Registers::MAX_FORWARD_SPEED));
		input.angular_velocity = -1 * clamp(steering * config.get<float>(Registers::TURN_SPEED), -config.get<float>(Registers::MAX_TURN_SPEED), config.get<float>(Registers::MAX_TURN_SPEED));
		motorPublisher->publish(input);
	}

	rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motorPublisher;
	rclcpp::Subscription<autonav_msgs::msg::SteamInput>::SharedPtr steamSubscription;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speedSubscription;
	rclcpp::TimerBase::SharedPtr heartbeatTimer;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SteamJoyNode>());
	rclcpp::shutdown();
	return 0;
}
