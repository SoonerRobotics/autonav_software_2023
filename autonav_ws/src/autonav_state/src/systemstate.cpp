#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/srv/set_system_state.hpp"
#include "autonav_msgs/srv/set_device_state.hpp"
#include "autonav_msgs/msg/device_state.hpp"
#include "autonav_libs/common.h"
#include <chrono>

int g_displayForgiveness = 0;
int g_loggingForgiveness = 0;

bool isReadOnlyNode(int64_t id)
{
	return id == Autonav::hash("autonav_display")
		|| id == Autonav::hash("autonav_logging")
		|| id == Autonav::hash("autonav_logging_combined")
		|| id == Autonav::hash("autonav_filters")
		|| id == Autonav::hash("autonav_nav_astar")
		|| id == Autonav::hash("autonav_nav_resolver")
		|| id == Autonav::hash("autonav_serial_camera")
		|| id == Autonav::hash("autonav_manual_steamtranslator")
		|| id == Autonav::hash("autonav_vision_expandifier")
		|| id == Autonav::hash("autonav_vision_transformer");
}

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
		this->declare_parameter("required_nodes", std::vector<std::string>());
		m_isSimulator = this->get_parameter("is_simulator").as_bool();
		m_forcedState = this->get_parameter("forced_state").as_string();
		auto requiredNodes = this->get_parameter("required_nodes").as_string_array();
		for (auto& node : requiredNodes)
		{
			RCLCPP_INFO(this->get_logger(), "Adding required node: %s", node.c_str());
			m_requiredNodes.push_back(Autonav::hash(node));
		}
		estop = false;
		mobility = false;

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
			kill(getpid(), SIGKILL);
			return;
		}

		if (std::find(nodes.begin(), nodes.end(), "/autonav_logging") == nodes.end() && ++g_loggingForgiveness > 15)
		{
			setSystemState(Autonav::State::SystemState::SHUTDOWN);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			rclcpp::shutdown(nullptr, "Autonav logging node not found");
			kill(getpid(), SIGKILL);
			return;
		}
	}

	void setDeviceState(int64_t id, Autonav::State::DeviceState state)
	{
		m_deviceStates[id] = state;

		auto msg = autonav_msgs::msg::DeviceState();
		msg.device = id;
		msg.state = state;
		m_deviceStatePublisher->publish(msg);
	}

	void setSystemState(Autonav::State::SystemState state)
	{
		m_systemState = state;

		auto msg = autonav_msgs::msg::SystemState();
		msg.state = state;
		msg.estop = estop;
		msg.mobility = mobility;
		msg.is_simulator = this->get_parameter("is_simulator").as_bool();
		m_systemStatePublisher->publish(msg);
	}

	bool switchToManual()
	{
		setSystemState(Autonav::State::SystemState::MANUAL);

		// Switch all devices to operating if their state is ready
		for (auto& device : m_deviceStates)
		{
			if (device.second == Autonav::State::DeviceState::READY)
			{
				setDeviceState(device.first, Autonav::State::DeviceState::OPERATING);
			}
		}
		return true;
	}

	void set_system_state(const std::shared_ptr<autonav_msgs::srv::SetSystemState::Request> request, std::shared_ptr<autonav_msgs::srv::SetSystemState::Response> response)
	{
		if (request->state == m_systemState)
		{
			estop = request->estop;
			mobility = request->mobility;
			setSystemState(m_systemState);
			response->ok = true;
			return;
		}

		if (request->state == Autonav::State::SystemState::MANUAL)
		{
			response->ok = switchToManual();
			return;
		}

		if (request->state == Autonav::State::SystemState::AUTONOMOUS)
		{
			// response->ok = trySwitchAutonomous();
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
		// switchDisabled();
		response->ok = true;
	}

	void set_device_state(const std::shared_ptr<autonav_msgs::srv::SetDeviceState::Request> request, std::shared_ptr<autonav_msgs::srv::SetDeviceState::Response> response)
	{
		auto device = request->device;
		auto state = (Autonav::State::DeviceState)request->state;

		if (m_deviceStates.find(device) == m_deviceStates.end())
		{
			m_deviceStates[device] = Autonav::State::DeviceState::OFF;
		}

		bool allLoaded = true;
		for(auto& device : m_requiredNodes)
		{
			if(m_deviceStates[device] != Autonav::State::DeviceState::READY && m_deviceStates[device] != Autonav::State::DeviceState::OPERATING)
			{
				allLoaded = false;
				break;
			}
		}

		// Get index in the required nodes array, -1 means not found
		int index = -1;
		for (int i = 0; i < m_requiredNodes.size(); i++)
		{
			if (m_requiredNodes[i] == device)
			{
				index = i;
				break;
			}
		}

		if (!allLoaded && index == -1)
		{
			response->ok = false;
			return;
		}

		// Go through required nodes, ensure they load in order from 0 to n (n being the last node)
		if (m_deviceStates[device] == Autonav::State::DeviceState::OFF && !allLoaded)
		{
			// Check if all previous nodes are ready
			for (int i = 0; i < index; i++)
			{
				if (m_deviceStates[m_requiredNodes[i]] != Autonav::State::DeviceState::READY)
				{
					response->ok = false;
					return;
				}
			}
		}

		if (state == Autonav::State::DeviceState::READY && m_forcedState == "autonomous" && !isReadOnlyNode(request->device))
		{
			// RCLCPP_INFO(this->get_logger(), "[0] Setting device state for %d to %d", device, Autonav::State::DeviceState::OPERATING);
			setDeviceState(device, Autonav::State::DeviceState::OPERATING);
		} else {
			// RCLCPP_INFO(this->get_logger(), "[1] Setting device state for %d to %d", device, state);
			setDeviceState(device, state);
		}
		response->ok = true;
		
		auto sys_msg = autonav_msgs::msg::SystemState();
		sys_msg.state = allLoaded ? (uint8_t)m_systemState : (uint8_t)Autonav::State::SystemState::DISABLED;
		sys_msg.is_simulator = this->get_parameter("is_simulator").as_bool();
		m_systemStatePublisher->publish(sys_msg);

		// if(m_systemState == Autonav::State::SystemState::MANUAL && !trySwitchManual(true) && m_forcedState != "manual")
		// {
		// 	switchDisabled();
		// 	return;
		// }

		// if(m_systemState == Autonav::State::SystemState::AUTONOMOUS && !trySwitchAutonomous(true) && m_forcedState != "autonomous")
		// {
		// 	switchDisabled();
		// 	return;
		// }	
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
	bool estop;
	bool mobility;
	std::vector<int64_t> m_requiredNodes;
	std::map<int64_t, Autonav::State::DeviceState> m_deviceStates = {};
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StateSystemNode>());
	rclcpp::shutdown();
}