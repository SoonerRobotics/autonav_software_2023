#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/srv/set_system_state.hpp"
#include "autonav_msgs/srv/set_device_state.hpp"
#include "autonav_msgs/msg/device_state.hpp"
#include "autonav_libs/common.h"

std::map<Autonav::Device, Autonav::State::DeviceState> deviceStates = {{
	{Autonav::Device::DISPLAY_NODE, Autonav::State::DeviceState::OFF},
	{Autonav::Device::LOGGING, Autonav::State::DeviceState::OFF},
	{Autonav::Device::MANUAL_CONTROL_STEAM, Autonav::State::DeviceState::OFF},
	{Autonav::Device::MANUAL_CONTROL_XBOX, Autonav::State::DeviceState::OFF},
	{Autonav::Device::SERIAL_CAN, Autonav::State::DeviceState::OFF},
	{Autonav::Device::SERIAL_IMU, Autonav::State::DeviceState::OFF},
	{Autonav::Device::STEAM_TRANSLATOR, Autonav::State::DeviceState::OFF}
}};

Autonav::State::SystemState systemState = Autonav::State::SystemState::DISABLED;
std::shared_ptr<rclcpp::Publisher<autonav_msgs::msg::DeviceState>> deviceStatePublisher;
std::shared_ptr<rclcpp::Publisher<autonav_msgs::msg::SystemState>> systemStatePublisher;

bool canSwitchToManual()
{
	// return true;
	return (
		deviceStates[Autonav::Device::DISPLAY_NODE] >= Autonav::State::DeviceState::READY &&
		(
			deviceStates[Autonav::Device::MANUAL_CONTROL_XBOX] >= Autonav::State::DeviceState::READY ||
			(
				deviceStates[Autonav::Device::MANUAL_CONTROL_STEAM] >= Autonav::State::DeviceState::READY &&
				deviceStates[Autonav::Device::STEAM_TRANSLATOR] >= Autonav::State::DeviceState::READY
			)
		) &&
		deviceStates[Autonav::Device::SERIAL_CAN] >= Autonav::State::DeviceState::READY
	);
}

bool canSwitchToAutonomous()
{
	return (
		deviceStates[Autonav::Device::DISPLAY_NODE] >= Autonav::State::DeviceState::READY &&
		deviceStates[Autonav::Device::LOGGING] >= Autonav::State::DeviceState::READY &&
		deviceStates[Autonav::Device::SERIAL_CAN] >= Autonav::State::DeviceState::READY &&
		deviceStates[Autonav::Device::SERIAL_IMU] >= Autonav::State::DeviceState::READY
	);
}

void set_system_state(const std::shared_ptr<autonav_msgs::srv::SetSystemState::Request> request, std::shared_ptr<autonav_msgs::srv::SetSystemState::Response> response)
{
	auto state = (Autonav::State::SystemState)request->state;
	
	if(state == Autonav::State::SystemState::MANUAL && canSwitchToManual())
	{
		systemState = state;
		response->ok = true;

		auto msg = autonav_msgs::msg::SystemState();
		msg.state = Autonav::State::SystemState::MANUAL;
		systemStatePublisher->publish(msg);
		return;
	}

	if(state == Autonav::State::SystemState::AUTONOMOUS && canSwitchToAutonomous())
	{
		systemState = state;
		response->ok = true;

		auto msg = autonav_msgs::msg::SystemState();
		msg.state = Autonav::State::SystemState::AUTONOMOUS;
		systemStatePublisher->publish(msg);
		return;
	}

	response->ok = false;
}

void set_device_state(const std::shared_ptr<autonav_msgs::srv::SetDeviceState::Request> request, std::shared_ptr<autonav_msgs::srv::SetDeviceState::Response> response)
{
	auto state = (Autonav::State::DeviceState)request->state;
	auto device = (Autonav::Device)request->device;

	if (state == Autonav::State::DeviceState::ALIVE)
	{
		auto msg = autonav_msgs::msg::DeviceState();
		msg.device = (uint8_t)device;
		msg.state = Autonav::State::DeviceState::STANDBY;
		deviceStatePublisher->publish(msg);
		deviceStates[device] = Autonav::State::DeviceState::STANDBY;
		response->ok = true;
		RCLCPP_INFO(rclcpp::get_logger("autonav_state_system"), "[ALIVE] Setting device state for %d to %d", device, state);
		return;
	}

	RCLCPP_INFO(rclcpp::get_logger("autonav_state_system"), "Setting device state for %d to %d", device, state);
	deviceStates[device] = state;
	auto msg = autonav_msgs::msg::DeviceState();
	msg.device = (uint8_t)device;
	msg.state = state;
	deviceStatePublisher->publish(msg);

	response->ok = true;
}

void on_timer_tick()
{
	// Publish all device states and system state
	for (auto deviceState : deviceStates)
	{
		auto msg = autonav_msgs::msg::DeviceState();
		msg.device = (uint8_t)deviceState.first;
		msg.state = deviceState.second;
		deviceStatePublisher->publish(msg);
	}

	auto msg = autonav_msgs::msg::SystemState();
	msg.state = systemState;
	systemStatePublisher->publish(msg);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	auto node = rclcpp::Node::make_shared("autonav_state_system");
	auto sssService = node->create_service<autonav_msgs::srv::SetSystemState>("/autonav/state/set_system_state", &set_system_state);
	auto sdsService = node->create_service<autonav_msgs::srv::SetDeviceState>("/autonav/state/set_device_state", &set_device_state);
	deviceStatePublisher = node->create_publisher<autonav_msgs::msg::DeviceState>("/autonav/state/device", 10);
	systemStatePublisher = node->create_publisher<autonav_msgs::msg::SystemState>("/autonav/state/system", 10);
	auto timer = node->create_wall_timer(std::chrono::milliseconds(1000), &on_timer_tick);
	rclcpp::spin(node);
	rclcpp::shutdown();
}