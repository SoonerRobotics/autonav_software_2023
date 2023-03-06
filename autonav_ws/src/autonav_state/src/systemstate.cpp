#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/srv/set_system_state.hpp"
#include "autonav_msgs/srv/set_device_state.hpp"
#include "autonav_msgs/msg/device_state.hpp"
#include "autonav_libs/common.h"
#include <chrono>

std::map<Autonav::Device, Autonav::State::DeviceState> g_deviceStates = {{
	{Autonav::Device::DISPLAY_NODE, Autonav::State::DeviceState::OFF},
	{Autonav::Device::LOGGING, Autonav::State::DeviceState::OFF},
	{Autonav::Device::MANUAL_CONTROL_STEAM, Autonav::State::DeviceState::OFF},
	{Autonav::Device::MANUAL_CONTROL_XBOX, Autonav::State::DeviceState::OFF},
	{Autonav::Device::SERIAL_CAN, Autonav::State::DeviceState::OFF},
	{Autonav::Device::SERIAL_IMU, Autonav::State::DeviceState::OFF},
	{Autonav::Device::STEAM_TRANSLATOR, Autonav::State::DeviceState::OFF}
}};

Autonav::State::SystemState g_systemState = Autonav::State::SystemState::DISABLED;
std::shared_ptr<rclcpp::Publisher<autonav_msgs::msg::DeviceState>> g_deviceStatePublisher;
std::shared_ptr<rclcpp::Publisher<autonav_msgs::msg::SystemState>> g_systemStatePublisher;

void setDeviceState(Autonav::Device device, Autonav::State::DeviceState state)
{
	g_deviceStates[device] = state;

	auto msg = autonav_msgs::msg::DeviceState();
	msg.device = (uint8_t)device;
	msg.state = state;
	g_deviceStatePublisher->publish(msg);
}

void setSystemState(Autonav::State::SystemState state)
{
	g_systemState = state;

	auto msg = autonav_msgs::msg::SystemState();
	msg.state = state;
	g_systemStatePublisher->publish(msg);
}

bool isReadonlyNode(Autonav::Device device)
{
	return device == Autonav::Device::DISPLAY_NODE 
		|| device == Autonav::Device::LOGGING
		|| device == Autonav::Device::STEAM_TRANSLATOR;
}

bool trySwitchManual(bool dontSwitch = false)
{
	auto canSwitch = (
		g_deviceStates[Autonav::Device::DISPLAY_NODE] >= Autonav::State::DeviceState::READY &&
		(
			g_deviceStates[Autonav::Device::MANUAL_CONTROL_XBOX] >= Autonav::State::DeviceState::READY ||
			(
				g_deviceStates[Autonav::Device::MANUAL_CONTROL_STEAM] >= Autonav::State::DeviceState::READY &&
				g_deviceStates[Autonav::Device::STEAM_TRANSLATOR] >= Autonav::State::DeviceState::READY
			)
		) &&
		g_deviceStates[Autonav::Device::SERIAL_CAN] >= Autonav::State::DeviceState::READY
	);

	if (dontSwitch)
	{
		return canSwitch;
	}

	if (!canSwitch)
	{
		return false;
	}

	setSystemState(Autonav::State::SystemState::MANUAL);
	setDeviceState(Autonav::Device::SERIAL_CAN, Autonav::State::DeviceState::OPERATING);
	if (g_deviceStates[Autonav::Device::MANUAL_CONTROL_XBOX] >= Autonav::State::DeviceState::READY)
	{
		setDeviceState(Autonav::Device::MANUAL_CONTROL_XBOX, Autonav::State::DeviceState::OPERATING);
		return true;
	}

	setDeviceState(Autonav::Device::MANUAL_CONTROL_STEAM, Autonav::State::DeviceState::OPERATING);
	setDeviceState(Autonav::Device::STEAM_TRANSLATOR, Autonav::State::DeviceState::OPERATING);
	return true;
}

void switchDisabled()
{
	setSystemState(Autonav::State::SystemState::DISABLED);
	for (auto deviceState : g_deviceStates)
	{
		if (deviceState.second == Autonav::State::DeviceState::OPERATING && !isReadonlyNode(deviceState.first))
		{
			setDeviceState(deviceState.first, Autonav::State::DeviceState::READY);
		}
	}
}

bool trySwitchAutonomous(bool dontSwitch = false)
{
	auto canSwitch = (
		g_deviceStates[Autonav::Device::DISPLAY_NODE] >= Autonav::State::DeviceState::READY &&
		g_deviceStates[Autonav::Device::LOGGING] >= Autonav::State::DeviceState::READY &&
		g_deviceStates[Autonav::Device::SERIAL_CAN] >= Autonav::State::DeviceState::READY &&
		g_deviceStates[Autonav::Device::SERIAL_IMU] >= Autonav::State::DeviceState::READY
	);

	if (dontSwitch)
	{
		return canSwitch;
	}

	if (!canSwitch)
	{
		return false;
	}

	setSystemState(Autonav::State::SystemState::AUTONOMOUS);
	setDeviceState(Autonav::Device::LOGGING, Autonav::State::DeviceState::OPERATING);
	setDeviceState(Autonav::Device::SERIAL_CAN, Autonav::State::DeviceState::OPERATING);
	setDeviceState(Autonav::Device::SERIAL_IMU, Autonav::State::DeviceState::OPERATING);
	return true;
}

void set_system_state(const std::shared_ptr<autonav_msgs::srv::SetSystemState::Request> request, std::shared_ptr<autonav_msgs::srv::SetSystemState::Response> response)
{
	if (request->state == g_systemState)
	{
		response->ok = false;
		return;
	}

	if (request->state == Autonav::State::SystemState::MANUAL)
	{
		response->ok = trySwitchManual();
		return;
	}

	if (request->state == Autonav::State::SystemState::AUTONOMOUS)
	{
		response->ok = trySwitchAutonomous();
		return;
	}

	if (request->state == Autonav::State::SystemState::SHUTDOWN)
	{
		response->ok = true;
		setSystemState(Autonav::State::SystemState::SHUTDOWN);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		rclcpp::shutdown(nullptr, "Shutdown requested");
		return;
	}

	// This is bad, we should also be doing an if statement just incase we add more states later on
	switchDisabled();
	response->ok = true;
}

void set_device_state(const std::shared_ptr<autonav_msgs::srv::SetDeviceState::Request> request, std::shared_ptr<autonav_msgs::srv::SetDeviceState::Response> response)
{
	auto device = (Autonav::Device)request->device;
	auto state = (Autonav::State::DeviceState)request->state;

	auto displayState = g_deviceStates[Autonav::Device::DISPLAY_NODE];
	if (displayState != Autonav::State::DeviceState::OPERATING && device != Autonav::Device::DISPLAY_NODE)
	{
		response->ok = false;
		return;
	}

	setDeviceState(device, state);
	response->ok = true;

	if(g_systemState == Autonav::State::SystemState::MANUAL && !trySwitchManual(true))
	{
		switchDisabled();
		return;
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	auto node = rclcpp::Node::make_shared("autonav_state_system");
	auto sssService = node->create_service<autonav_msgs::srv::SetSystemState>("/autonav/state/set_system_state", &set_system_state);
	auto sdsService = node->create_service<autonav_msgs::srv::SetDeviceState>("/autonav/state/set_device_state", &set_device_state);
	g_deviceStatePublisher = node->create_publisher<autonav_msgs::msg::DeviceState>("/autonav/state/device", 10);
	g_systemStatePublisher = node->create_publisher<autonav_msgs::msg::SystemState>("/autonav/state/system", 10);

	rclcpp::spin(node);
	rclcpp::shutdown();
}