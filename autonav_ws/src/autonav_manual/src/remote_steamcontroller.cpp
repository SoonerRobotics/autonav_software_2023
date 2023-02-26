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

class SteamNode : public Autonav::ROS::AutoNode
{
public:
	SteamNode() : AutoNode(Autonav::Device::MANUAL_CONTROL, "remote_steamcontroller")
	{
		subscription_ = this->create_subscription<autonav_msgs::msg::SteamInput>("/autonav/joy/steam", 20, std::bind(&SteamNode::on_steam_received, this, _1));
		motor_publisher = this->create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 20);
		timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / 20), std::bind(&SteamNode::on_timer_elapsed, this));

		this->_config.write(Registers::TIMEOUT_DELAY, 500);
		this->_config.write(Registers::STEERING_DEADZONE, 0.35f);
		this->_config.write(Registers::THROTTLE_DEADZONE, 0.1f);
		this->_config.write(Registers::MAX_SPEED, 2.0f);
		this->_config.write(Registers::SPEED_OFFSET, 0.6f);
		
		this->setDeviceState(Autonav::State::DeviceState::OPERATING);
	}

private:
	void on_timer_elapsed()
	{
		long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		if (currentTime - lastMessageTime < this->_config.read<int32_t>(Registers::TIMEOUT_DELAY) || this->_systemState != Autonav::State::SystemState::MANUAL)
		{
			return;
		}

		autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
		package.left_motor = 0;
		package.right_motor = 0;
		motor_publisher->publish(package);
	}

	void on_steam_received(const autonav_msgs::msg::SteamInput &msg)
	{
		lastMessageTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		if (this->_systemState != Autonav::State::SystemState::MANUAL)
		{
			return;
		}

		float throttle = 0;
		float steering = 0;
		const float deadzone = this->_config.read<float>(Registers::THROTTLE_DEADZONE);
		const float maxSpeed = this->_config.read<float>(Registers::MAX_SPEED);
		const float steeringVoid = this->_config.read<float>(Registers::STEERING_DEADZONE);
		const float offset = this->_config.read<float>(Registers::SPEED_OFFSET);

		if (abs(msg.ltrig) > deadzone || abs(msg.rtrig) > deadzone)
		{
			throttle = (1 - -msg.rtrig) * maxSpeed * 0.8;
			throttle = throttle - (1 - -msg.ltrig) * maxSpeed * 0.8;
		}

		if(abs(msg.rpad_x) > steeringVoid)
		{
			float real = abs(msg.rpad_x) - steeringVoid;
			if(msg.rpad_x < 0)
			{
				real = -real;
			}

			steering = -real * maxSpeed;
		}

		autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
		package.left_motor = clamp(throttle - steering * offset, -maxSpeed, maxSpeed);
		package.right_motor = clamp(throttle + steering * offset, -maxSpeed, maxSpeed);
		motor_publisher->publish(package);
	}

	rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motor_publisher;
	rclcpp::Subscription<autonav_msgs::msg::SteamInput>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SteamNode>());
	rclcpp::shutdown();
	return 0;
}