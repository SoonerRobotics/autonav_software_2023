#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_libs/autonav.h"

static Autonav::ConBus::Controller controller;

class ExampleNode : public rclcpp::Node
{
public:
	ExampleNode()
		: Node("example_node")
	{
		_publisher = this->create_publisher<autonav_msgs::msg::ConBusInstruction>("/autonav/conbus", 10);
		_controller = Autonav::ConBus::Controller(0, _publisher);
		_subscription = this->create_subscription<autonav_msgs::msg::ConBusInstruction>("/autonav/conbus", 10, std::bind(&Autonav::ConBus::Controller::onInstruction, &_controller, std::placeholders::_1));
	}

private:
	rclcpp::Publisher<autonav_msgs::msg::ConBusInstruction>::SharedPtr _publisher;
	rclcpp::Subscription<autonav_msgs::msg::ConBusInstruction>::SharedPtr _subscription;
	Autonav::ConBus::Controller _controller;
};

int main(int argc, char *argv[])
{
	// Write example instructions for "ros2 topic pub" to set and get a register
	// ros2 topic pub --once /autonav/conbus autonav_msgs/msg/ConBusInstruction "{opcode: 2, device: 0, address: 0, data: [2, 5]}"
	// ros2 topic pub --once /autonav/conbus autonav_msgs/msg/ConBusInstruction "{opcode: 0, device: 0, address: 0, data: [0, 0]}"

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ExampleNode>());
	rclcpp::shutdown();
	return 0;
}