#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/steam_input.hpp"
#include "autonav_libs/common.h"

using std::placeholders::_1;

long lastMessageTime = 0;

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
	TIMEOUT_DELAY = 0,
	STEERING_DEADZONE = 1,
	THROTTLE_DEADZONE = 2,
	MAX_SPEED = 3,
	SPEED_OFFSET = 4
};

class JoyNode : public Autonav::ROS::AutoNode
{
public:
	JoyNode() : AutoNode(Autonav::Device::MANUAL_CONTROL_STEAM, "remote_steamcontroller") {}

	void setup() override
	{
		m_steamSubscription = create_subscription<autonav_msgs::msg::SteamInput>("/autonav/joy/steam", 20, std::bind(&JoyNode::on_steam_received, this, _1));
		m_motorPublisher = create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 20);

		config.write(Registers::TIMEOUT_DELAY, 500);
		config.write(Registers::STEERING_DEADZONE, 0.35f);
		config.write(Registers::THROTTLE_DEADZONE, 0.1f);
		config.write(Registers::MAX_SPEED, 2.2f);
		config.write(Registers::SPEED_OFFSET, 0.6f);

		setDeviceState(Autonav::State::DeviceState::READY);
	}

	void operate() override
	{
		m_timer = this->create_wall_timer(std::chrono::milliseconds(1000 / 20), std::bind(&JoyNode::on_timer_elapsed, this));
	}

	void on_timer_elapsed()
	{
		long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		if (currentTime - lastMessageTime < config.read<int32_t>(Registers::TIMEOUT_DELAY) || getSystemState() != Autonav::State::SystemState::MANUAL)
		{
			return;
		}

		autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
		package.left_motor = 0;
		package.right_motor = 0;
		m_motorPublisher->publish(package);
	}

	void on_steam_received(const autonav_msgs::msg::SteamInput &msg)
	{
		lastMessageTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		if (getSystemState() != Autonav::State::SystemState::MANUAL)
		{
			return;
		}

		float throttle = 0;
		float steering = 0;
		const float deadzone = config.read<float>(Registers::THROTTLE_DEADZONE);
		const float maxSpeed = config.read<float>(Registers::MAX_SPEED);
		const float steeringVoid = config.read<float>(Registers::STEERING_DEADZONE);
		const float offset = config.read<float>(Registers::SPEED_OFFSET);

		if (abs(msg.rtrig) > deadzone || abs(msg.ltrig) > deadzone)
		{
			throttle = (1 - msg.ltrig) * maxSpeed * 0.8;
			throttle = throttle - (1 - msg.rtrig) * maxSpeed * 0.8;
		}

		if (abs(msg.lpad_x) > steeringVoid)
		{
			float real = abs(msg.lpad_x) - steeringVoid;
			if (msg.lpad_x < 0)
			{
				real = -real;
			}

			steering = -real * maxSpeed;
		}

		autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
		package.left_motor = clamp(-throttle + steering * offset, -maxSpeed, maxSpeed);
		package.right_motor = clamp(-throttle - steering * offset, -maxSpeed, maxSpeed);
		m_motorPublisher->publish(package);
	}

	rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr m_motorPublisher;
	rclcpp::Subscription<autonav_msgs::msg::SteamInput>::SharedPtr m_steamSubscription;
	rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JoyNode>());
	rclcpp::shutdown();
	return 0;
}