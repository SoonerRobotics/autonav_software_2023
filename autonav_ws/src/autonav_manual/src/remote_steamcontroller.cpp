#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/steam_input.hpp"
#include "std_msgs/msg/float32.hpp"
#include "scr_core/node.h"
#include "scr_core/device_state.h"

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
	TIMEOUT_DELAY = 0,
	STEERING_DEADZONE = 1,
	THROTTLE_DEADZONE = 2,
	MAX_SPEED = 3,
	MAX_TURN_SPEED = 4
};

class JoyNode : public SCR::Node
{
public:
	JoyNode() : SCR::Node("autonav_manual_steamcontroller") {}

	void configure() override
	{
		m_steamSubscription = create_subscription<autonav_msgs::msg::SteamInput>("/autonav/joy/steam", 20, std::bind(&JoyNode::on_steam_received, this, _1));
		m_motorPublisher = create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 20);
		m_steeringSubscription = create_subscription<std_msgs::msg::Float32>("/autonav/speed", 20, std::bind(&JoyNode::onSteeringReceived, this, _1));

		config.set(Registers::TIMEOUT_DELAY, 500);
		config.set(Registers::STEERING_DEADZONE, 0.04f);
		config.set(Registers::THROTTLE_DEADZONE, 0.04f);
		config.set(Registers::MAX_SPEED, 1.0f);
		config.set(Registers::MAX_TURN_SPEED, 0.6f);

		m_timer = this->create_wall_timer(std::chrono::milliseconds(1000 / 20), std::bind(&JoyNode::on_timer_elapsed, this));
		setDeviceState(SCR::DeviceState::OPERATING);
	}

	void onSteeringReceived(const std_msgs::msg::Float32::SharedPtr msg)
	{
		speed = msg->data;
	}

	void on_timer_elapsed()
	{
		long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		if (currentTime - lastMessageTime < config.get<int>(Registers::TIMEOUT_DELAY) || getSystemState().state != SCR::SystemState::MANUAL)
		{
			return;
		}

		// autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
		// package.forward_velocity = 0;
		// package.angular_velocity = 0;
		// m_motorPublisher->publish(package);
	}

	void on_steam_received(const autonav_msgs::msg::SteamInput &msg)
	{
		lastMessageTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		if (getSystemState().state != SCR::SystemState::MANUAL || getDeviceState() != SCR::DeviceState::OPERATING)
		{
			return;
		}

		float throttle = 0;
		float steering = 0;
		const float deadzone = config.get<float>(Registers::THROTTLE_DEADZONE);
		float maxSpeed = config.get<float>(Registers::MAX_SPEED);
		float maxTurnSpeed = config.get<float>(Registers::MAX_TURN_SPEED);
		const float steeringVoid = config.get<float>(Registers::STEERING_DEADZONE);

		if (abs(msg.rtrig) > deadzone || abs(msg.ltrig) > deadzone)
		{
			throttle = (1 - msg.rtrig) * maxSpeed * 0.95;
			throttle = throttle - (1 - msg.ltrig) * maxSpeed * 0.95;
		}

		maxSpeed = speed;
		maxTurnSpeed = speed;
		if (abs(msg.lpad_x) > steeringVoid)
		{
			float real = abs(msg.lpad_x) - steeringVoid;
			if (msg.lpad_x < 0)
			{
				real = -real;
			}

			steering = real * maxSpeed;
		}

		auto forward_speed = clamp(-throttle * 4, -maxSpeed, maxSpeed);
		auto turn_angle_rads = clamp(steering, -maxSpeed, maxSpeed);
		autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
		package.forward_velocity = forward_speed;
		auto turn_angle_rads_counter_clockwise = -turn_angle_rads;
		package.angular_velocity = clamp(turn_angle_rads_counter_clockwise * 4, -maxTurnSpeed, maxTurnSpeed);

		lastForwardSpeed = package.forward_velocity;
		lastTurnSpeed = package.angular_velocity;

		m_motorPublisher->publish(package);
	}

	rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr m_motorPublisher;
	rclcpp::Subscription<autonav_msgs::msg::SteamInput>::SharedPtr m_steamSubscription;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_steeringSubscription;
	rclcpp::TimerBase::SharedPtr m_timer;
	float speed = 0.3f;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JoyNode>());
	rclcpp::shutdown();
	return 0;
}