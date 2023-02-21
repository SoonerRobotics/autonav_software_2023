#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autonav_msgs/msg/motor_input.hpp"

using std::placeholders::_1;

long lastMessageTime = 0;

namespace AutonavConstants
{
	const int THROTTLE_KEY = 3;
	const int STEERING_KEY = 0;
	const int DEADZONE = 0.15;
	const int MAX_SPEED = 1.4;
	const int TIMER_INTERVAL = 50;
	const long TIMEOUT = 1000;
}

float clamp(float value, float min, float max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

class JoySubscriber : public rclcpp::Node
{
public:
	JoySubscriber() : Node("autonav_manual_remote")
	{
		subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoySubscriber::on_joy_received, this, _1));
		motor_publisher = this->create_publisher<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 10);
		timer_ = this->create_wall_timer(std::chrono::milliseconds(AutonavConstants::TIMER_INTERVAL), std::bind(&JoySubscriber::on_timer_elapsed, this));
	}

private:
	void on_timer_elapsed() const
	{
		long currentTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		if (currentTime - lastMessageTime < AutonavConstants::TIMEOUT)
		{
			return;
		}

		autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
		package.left_motor = 0;
		package.right_motor = 0;
		motor_publisher->publish(package);
	}

	void on_joy_received(const sensor_msgs::msg::Joy &msg) const
	{
		lastMessageTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

		float throttle = 0;
		float steering = 0;

		autonav_msgs::msg::MotorInput package = autonav_msgs::msg::MotorInput();
		if (abs(msg.axes[AutonavConstants::THROTTLE_KEY]) > AutonavConstants::DEADZONE)
		{
			throttle = msg.axes[AutonavConstants::THROTTLE_KEY] * AutonavConstants::MAX_SPEED * 0.75;
		}

		if (abs(msg.axes[AutonavConstants::STEERING_KEY]) > AutonavConstants::DEADZONE)
		{
			steering = msg.axes[AutonavConstants::STEERING_KEY] * AutonavConstants::MAX_SPEED;
		}

		package.left_motor = clamp(throttle - steering * 0.6, -AutonavConstants::MAX_SPEED, AutonavConstants::MAX_SPEED);
		package.right_motor = clamp(throttle + steering * 0.6, -AutonavConstants::MAX_SPEED, AutonavConstants::MAX_SPEED);
		motor_publisher->publish(package);
	}

	rclcpp::Publisher<autonav_msgs::msg::MotorInput>::SharedPtr motor_publisher;
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JoySubscriber>());
	rclcpp::shutdown();
	return 0;
}