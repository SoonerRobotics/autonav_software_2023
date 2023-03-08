#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/srv/set_system_state.hpp"
#include "autonav_msgs/srv/set_device_state.hpp"
#include "autonav_msgs/msg/device_state.hpp"
#include "autonav_libs/common.h"
#include <chrono>

int g_displayForgiveness = 0;
int g_loggingForgiveness = 0;

class StateSystemNode : public rclcpp::Node
{
public:
	StateSystemNode() : Node("autonav_state_system")
	{
		m_sssService = this->create_service<autonav_msgs::srv::SetSystemState>("/autonav/state/set_system_state", std::bind(&StateSystemNode::set_system_state, this, std::placeholders::_1, std::placeholders::_2));
		m_sdsService = this->create_service<autonav_msgs::srv::SetDeviceState>("/autonav/state/set_device_state", std::bind(&StateSystemNode::set_device_state, this, std::placeholders::_1, std::placeholders::_2));
		m_deviceStatePublisher = this->create_publisher<autonav_msgs::msg::DeviceState>("/autonav/state/device", 10);
		m_systemStatePublisher = this->create_publisher<autonav_msgs::msg::SystemState>("/autonav/state/system", 10);
		m_timer = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&StateSystemNode::on_requires_timer_tick, this));
	
		// Declare is_simulator ros parameter
		this->declare_parameter("is_simulator", false);
		this->declare_parameter("forced_state", "");
		m_isSimulator = this->get_parameter("is_simulator").as_bool();
		m_forcedState = this->get_parameter("forced_state").as_string();

		if(m_forcedState == "autonomous") {
			m_systemState = Autonav::State::SystemState::AUTONOMOUS;
		}

		if(m_forcedState == "manual") {
			m_systemState = Autonav::State::SystemState::MANUAL;
		}

		RCLCPP_INFO(this->get_logger(), "Is Simulator -> %s", m_isSimulator ? "true" : "false");
	}

private:
	void on_requires_timer_tick()
	{
		auto nodes = this->get_node_names();
		if (std::find(nodes.begin(), nodes.end(), "/autonav_display") == nodes.end() && ++g_displayForgiveness > 15)
		{
			setSystemState(Autonav::State::SystemState::SHUTDOWN);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			rclcpp::shutdown(nullptr, "Autonav display node not found");
			return;
		}

		if (std::find(nodes.begin(), nodes.end(), "/autonav_logging") == nodes.end() && ++g_loggingForgiveness > 15)
		{
			setSystemState(Autonav::State::SystemState::SHUTDOWN);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			rclcpp::shutdown(nullptr, "Autonav logging node not found");
			return;
		}
	}

	void setDeviceState(Autonav::Device device, Autonav::State::DeviceState state)
	{
		m_deviceStates[device] = state;

		auto msg = autonav_msgs::msg::DeviceState();
		msg.device = (uint8_t)device;
		msg.state = state;
		m_deviceStatePublisher->publish(msg);
	}

	void setSystemState(Autonav::State::SystemState state)
	{
		m_systemState = state;

		auto msg = autonav_msgs::msg::SystemState();
		msg.state = state;
		msg.is_simulator = this->get_parameter("is_simulator").as_bool();
		m_systemStatePublisher->publish(msg);
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
			(m_deviceStates[Autonav::Device::DISPLAY_NODE] >= Autonav::State::DeviceState::READY) &&
			(
				(m_deviceStates[Autonav::Device::MANUAL_CONTROL_XBOX] >= Autonav::State::DeviceState::READY) ||
				(
					(m_deviceStates[Autonav::Device::MANUAL_CONTROL_STEAM] >= Autonav::State::DeviceState::READY) &&
					(m_deviceStates[Autonav::Device::STEAM_TRANSLATOR] >= Autonav::State::DeviceState::READY)
				)
			) &&
			(
				(m_deviceStates[Autonav::Device::SERIAL_CAN] >= Autonav::State::DeviceState::READY) ||
				this->get_parameter("is_simulator").as_bool()
			)
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
		if (m_deviceStates[Autonav::Device::MANUAL_CONTROL_XBOX] >= Autonav::State::DeviceState::READY)
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
		for (auto deviceState : m_deviceStates)
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
			((m_deviceStates[Autonav::Device::DISPLAY_NODE] >= Autonav::State::DeviceState::READY) &&
			(m_deviceStates[Autonav::Device::LOGGING] >= Autonav::State::DeviceState::READY) &&
			(
				(m_deviceStates[Autonav::Device::SERIAL_CAN] >= Autonav::State::DeviceState::READY) &&
				(m_deviceStates[Autonav::Device::SERIAL_IMU] >= Autonav::State::DeviceState::READY)
			)) || this->get_parameter("is_simulator").as_bool()
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
		if (request->state == m_systemState)
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

		auto displayState = m_deviceStates[Autonav::Device::DISPLAY_NODE];
		if (displayState != Autonav::State::DeviceState::OPERATING && device != Autonav::Device::DISPLAY_NODE)
		{
			response->ok = false;
			return;
		}

		if (state == Autonav::State::DeviceState::READY && m_forcedState == "autonomous" && !isReadonlyNode(device))
		{
			setDeviceState(device, Autonav::State::DeviceState::OPERATING);
		} else {
			setDeviceState(device, state);
		}

		response->ok = true;
		
		auto sys_msg = autonav_msgs::msg::SystemState();
		sys_msg.state = (uint8_t)m_systemState;
		sys_msg.is_simulator = this->get_parameter("is_simulator").as_bool();
		m_systemStatePublisher->publish(sys_msg);

		if(m_systemState == Autonav::State::SystemState::MANUAL && !trySwitchManual(true) && m_forcedState != "manual")
		{
			switchDisabled();
			return;
		}

		if(m_systemState == Autonav::State::SystemState::AUTONOMOUS && !trySwitchAutonomous(true) && m_forcedState != "autonomous")
		{
			switchDisabled();
			return;
		}	
	}

private:
	std::shared_ptr<rclcpp::Publisher<autonav_msgs::msg::DeviceState>> m_deviceStatePublisher;
	std::shared_ptr<rclcpp::Publisher<autonav_msgs::msg::SystemState>> m_systemStatePublisher;
	std::shared_ptr<rclcpp::Service<autonav_msgs::srv::SetSystemState>> m_sssService;
	std::shared_ptr<rclcpp::Service<autonav_msgs::srv::SetDeviceState>> m_sdsService;
	rclcpp::TimerBase::SharedPtr m_timer;

	Autonav::State::SystemState m_systemState;
	std::string m_forcedState = "";
	bool m_isSimulator;
	std::map<Autonav::Device, Autonav::State::DeviceState> m_deviceStates = {{
		{Autonav::Device::DISPLAY_NODE, Autonav::State::DeviceState::OFF},
		{Autonav::Device::LOGGING, Autonav::State::DeviceState::OFF},
		{Autonav::Device::MANUAL_CONTROL_STEAM, Autonav::State::DeviceState::OFF},
		{Autonav::Device::MANUAL_CONTROL_XBOX, Autonav::State::DeviceState::OFF},
		{Autonav::Device::SERIAL_CAN, Autonav::State::DeviceState::OFF},
		{Autonav::Device::SERIAL_IMU, Autonav::State::DeviceState::OFF},
		{Autonav::Device::STEAM_TRANSLATOR, Autonav::State::DeviceState::OFF}
	}};
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StateSystemNode>());
	rclcpp::shutdown();
}