#include "scr_core/node.h"
#include <unistd.h>

#define UNUSED(expr) do { (void)(expr); } while (0)

namespace SCR
{
	Node::Node(std::string node_name) : rclcpp::Node(node_name)
	{
		id = SCR::hash(node_name);

		// State System
		deviceStates[getDeviceID()] = DeviceState::OFF;
		systemStateSubscriber = this->create_subscription<scr_msgs::msg::SystemState>("/scr/state/system", 20, std::bind(&Node::onSystemState, this, std::placeholders::_1));
		deviceStateSubscriber = this->create_subscription<scr_msgs::msg::DeviceState>("/scr/state/device", 20, std::bind(&Node::onDeviceState, this, std::placeholders::_1));
		resetSubscriber = this->create_subscription<std_msgs::msg::Empty>("/scr/reset", 20, std::bind(&Node::onResetInternal, this, std::placeholders::_1));
		deviceStateClient = this->create_client<scr_msgs::srv::SetDeviceState>("/scr/state/set_device_state");
		systemStateClient = this->create_client<scr_msgs::srv::SetSystemState>("/scr/state/set_system_state");
		resetPublisher = this->create_publisher<std_msgs::msg::Empty>("/scr/reset", 20);
		logPublisher = this->create_publisher<scr_msgs::msg::Log>("/scr/logging", 20);

		// Configuration
		auto configurationSubscriber = this->create_subscription<scr_msgs::msg::ConfigurationInstruction>("/scr/configuration", 20, std::bind(&Configuration::onConfigurationInstruction, &config, std::placeholders::_1));
		auto configurationPublisher = this->create_publisher<scr_msgs::msg::ConfigurationInstruction>("/scr/configuration", 20);
		config = Configuration(id, configurationSubscriber, configurationPublisher);

		// Performance
		auto performancePublisher = this->create_publisher<scr_msgs::msg::PerformanceResult>("/scr/performance", 20);
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

	int64_t Node::getDeviceID()
	{
		return id;
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
		request->device = getDeviceID();
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
		deviceStates[msg->device] = static_cast<DeviceState>(msg->state);
		if (msg->device != getDeviceID())
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

	scr_msgs::msg::SystemState Node::getSystemState()
	{
		return state;
	}

	DeviceState Node::getDeviceState()
	{
		return deviceStates[getDeviceID()];
	}

	DeviceState Node::getDeviceState(std::string id)
	{
		return deviceStates[SCR::hash(id)];
	}

	std::map<int64_t, DeviceState> Node::getDeviceStates()
	{
		return deviceStates;
	}
}