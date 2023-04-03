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

	void setDeviceState(int32_t id, Autonav::State::DeviceState state)
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
		msg.is_simulator = this->get_parameter("is_simulator").as_bool();
		m_systemStatePublisher->publish(msg);
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
			// response->ok = trySwitchManual();
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

		if (m_deviceStates.find(Autonav::hash("autonav_display")) == m_deviceStates.end())
		{
			m_deviceStates[Autonav::hash("autonav_display")] = Autonav::State::DeviceState::OFF;
		}
		
		auto displayState = m_deviceStates[Autonav::hash("autonav_display")];
		if (displayState != Autonav::State::DeviceState::OPERATING && device != Autonav::hash("autonav_display"))
		{
			response->ok = false;
			return;
		}

		if (state == Autonav::State::DeviceState::READY && m_forcedState == "autonomous" && device != Autonav::hash("autonav_display"))
		{
			setDeviceState(device, Autonav::State::DeviceState::OPERATING);
		} else {
			setDeviceState(device, state);
		}
		
		auto sys_msg = autonav_msgs::msg::SystemState();
		sys_msg.state = (uint8_t)m_systemState;
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
	std::map<int32_t, Autonav::State::DeviceState> m_deviceStates = {};
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StateSystemNode>());
	rclcpp::shutdown();
}