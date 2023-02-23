#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/steam_input.hpp"
#include "autonav_libs/autonav.h"

using std::placeholders::_1;

long lastMessageTime = 0;

enum RegisterID
{
	DEADZONE = 0,
	MAX_SPEED = 1,
	TIMEOUT = 2,
	TIMER_INTERVAL = 3,
	STEERING_VOID = 4
};

float clamp(float value, float min, float max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

class SteamNode : public Autonav::ConBus::ConbusNode
{
public:
	SteamNode() : Autonav::ConBus::ConbusNode(0, "autonav_manual_steamremote")
	{
		_controller.writeFloat(RegisterID::DEADZONE, 0.1);
		_controller.writeFloat(RegisterID::MAX_SPEED, 0.5);
		_controller.writeInt32(RegisterID::TIMEOUT, 1000);
		_controller.writeInt32(RegisterID::TIMER_INTERVAL, 100);
		_controller.writeFloat(RegisterID::STEERING_VOID, 0.1);

		subscription_ = this->create_subscription<autonav_msgs::msg::SteamInput>("/autonav/joy/steam", 20, std::bind(&SteamNode::on_steam_received, this, _1));
		motor_publisher = this->create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 20);
		timer_ = this->create_wall_timer(std::chrono::milliseconds(_controller.readInt32(RegisterID::TIMER_INTERVAL)), std::bind(&SteamNode::on_timer_elapsed, this));
	}

private:
	void on_timer_elapsed()
	{
		long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		if (currentTime - lastMessageTime < _controller.readInt32(RegisterID::TIMEOUT))
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

		float throttle = 0;
		float steering = 0;
		const float deadzone = _controller.readFloat(RegisterID::DEADZONE);
		const float maxSpeed = _controller.readFloat(RegisterID::MAX_SPEED);
		const float steeringVoid = _controller.readFloat(RegisterID::STEERING_VOID);

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
		package.left_motor = clamp(throttle - steering * 0.6, -maxSpeed, maxSpeed);
		package.right_motor = clamp(throttle + steering * 0.6, -maxSpeed, maxSpeed);
		motor_publisher->publish(package);
	}

	rclcpp::Publisher<autonav_msgs::msg::ConBusInstruction>::SharedPtr _conpublisher;
	rclcpp::Subscription<autonav_msgs::msg::ConBusInstruction>::SharedPtr _consubscriber;
	Autonav::ConBus::Controller _controller;

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