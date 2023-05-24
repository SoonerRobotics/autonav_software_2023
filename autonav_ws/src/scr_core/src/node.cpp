#include "scr_core/node.h"
#include <unistd.h>

#define UNUSED(expr) do { (void)(expr); } while (0)

namespace SCR
{
	Node::Node(std::string node_name) : rclcpp::Node(node_name)
	{
		// State System
		deviceStates[node_name] = DeviceState::OFF;
		systemStateSubscriber = this->create_subscription<scr_msgs::msg::SystemState>("/scr/state/system", 100, std::bind(&Node::onSystemState, this, std::placeholders::_1));
		deviceStateSubscriber = this->create_subscription<scr_msgs::msg::DeviceState>("/scr/state/device", 100, std::bind(&Node::onDeviceState, this, std::placeholders::_1));
		resetSubscriber = this->create_subscription<std_msgs::msg::Empty>("/scr/reset", 100, std::bind(&Node::onResetInternal, this, std::placeholders::_1));
		deviceStateClient = this->create_client<scr_msgs::srv::SetDeviceState>("/scr/state/set_device_state");
		systemStateClient = this->create_client<scr_msgs::srv::SetSystemState>("/scr/state/set_system_state");
		resetPublisher = this->create_publisher<std_msgs::msg::Empty>("/scr/reset", 100);
		logPublisher = this->create_publisher<scr_msgs::msg::Log>("/scr/logging", 100);

		// Configuration
		auto configurationSubscriber = this->create_subscription<scr_msgs::msg::ConfigurationInstruction>("/scr/configuration", 100, std::bind(&Configuration::onConfigurationInstruction, &config, std::placeholders::_1));
		auto configurationLoadSubscriber = this->create_subscription<std_msgs::msg::String>("/scr/configuration/preset", 100, std::bind(&Configuration::onPresetChanged, &config, std::placeholders::_1));
		auto configurationPublisher = this->create_publisher<scr_msgs::msg::ConfigurationInstruction>("/scr/configuration", 100);
		auto configurationLoadPublisher = this->create_publisher<std_msgs::msg::String>("/scr/configuration/load", 100);
		config = Configuration(node_name, configurationSubscriber, configurationPublisher, configurationLoadSubscriber, configurationLoadPublisher);

		// Performance
		auto performancePublisher = this->create_publisher<scr_msgs::msg::PerformanceResult>("/scr/performance", 100);
		performance = Performance(node_name, performancePublisher);
	}

	Node::~Node()
	{
	}

	void Node::configure()
	{
	}

	void Node::onReset()
	{
	}

	void Node::onResetInternal(const std_msgs::msg::Empty::SharedPtr msg)
	{
		UNUSED(msg);
		onReset();
	}

	void Node::setEStop(bool state)
	{
		auto request = std::make_shared<scr_msgs::srv::SetSystemState::Request>();
		request->state = this->state.state;
		request->estop = state;
		request->mobility = this->state.mobility;
		request->mode = this->state.mode;
		systemStateClient->async_send_request(request);
	}

	void Node::setMobility(bool state)
	{
		auto request = std::make_shared<scr_msgs::srv::SetSystemState::Request>();
		request->state = this->state.state;
		request->estop = this->state.estop;
		request->mobility = state;
		request->mode = this->state.mode;
		systemStateClient->async_send_request(request);
	}

	void Node::setSystemState(SystemState state)
	{
		// Send a system state client request
		auto request = std::make_shared<scr_msgs::srv::SetSystemState::Request>();
		request->state = static_cast<uint8_t>(state);
		request->estop = this->state.estop;
		request->mobility = this->state.mobility;
		request->mode = this->state.mode;
		systemStateClient->async_send_request(request);
	}

	void Node::setSystemMode(SystemMode mode)
	{
		auto request = std::make_shared<scr_msgs::srv::SetSystemState::Request>();
		request->state = this->state.state;
		request->estop = this->state.estop;
		request->mobility = this->state.mobility;
		request->mode = static_cast<uint8_t>(mode);
		systemStateClient->async_send_request(request);
	}

	void Node::setDeviceState(DeviceState state)
	{
		auto request = std::make_shared<scr_msgs::srv::SetDeviceState::Request>();
		request->device = this->get_name();
		request->state = static_cast<uint8_t>(state);
		deviceStateClient->async_send_request(request);
	}

	void Node::log(const std::string& message)
	{
		auto msg = scr_msgs::msg::Log();
		msg.data = message;
		msg.node = get_name();
		logPublisher->publish(msg);
	}

	void Node::reset()
	{
		auto msg = std_msgs::msg::Empty();
		resetPublisher->publish(msg);
	}

	void Node::onSystemState(const scr_msgs::msg::SystemState::SharedPtr msg)
	{
		if(msg->state == SystemState::SHUTDOWN)
		{
			kill(getpid(), SIGINT);
		}

		auto old = state;
		state = *msg;
		transition(old, state);
	}

	void Node::onDeviceState(const scr_msgs::msg::DeviceState::SharedPtr msg)
	{
		bool isNew = deviceStates.find(msg->device) == deviceStates.end();
		deviceStates[msg->device] = static_cast<DeviceState>(msg->state);
		onDeviceStateUpdated(msg, isNew && msg->state != SCR::DeviceState::OFF);
		if (msg->device != this->get_name())
		{
			return;
		}

		if (msg->state == static_cast<uint8_t>(DeviceState::STANDBY))
		{
			config.recache();
			configure();
			return;
		}
	}

	void Node::transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated)
	{
		UNUSED(old);
		UNUSED(updated);
		throw std::runtime_error("Transition function not overridden");
	}
	
	void Node::onDeviceStateUpdated(scr_msgs::msg::DeviceState::SharedPtr msg, bool isNew)
	{
	}

	scr_msgs::msg::SystemState Node::getSystemState()
	{
		return state;
	}

	DeviceState Node::getDeviceState()
	{
		return deviceStates[this->get_name()];
	}

	DeviceState Node::getDeviceState(std::string id)
	{
		return deviceStates[id];
	}

	std::map<std::string, DeviceState> Node::getDeviceStates()
	{
		return deviceStates;
	}
}