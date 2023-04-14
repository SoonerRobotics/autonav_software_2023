#include "autonav_libs/node.h"
#include <unistd.h>

#define UNUSED(expr) do { (void)(expr); } while (0)

namespace Autonav
{
	AutoNode::AutoNode(std::string node_name) : Node(node_name)
	{
		id = Autonav::hash(node_name);

		// State System
		deviceStates[getDeviceID()] = DeviceState::OFF;
		systemStateSubscriber = this->create_subscription<autonav_msgs::msg::SystemState>("/autonav/state/system", 10, std::bind(&AutoNode::onSystemState, this, std::placeholders::_1));
		deviceStateSubscriber = this->create_subscription<autonav_msgs::msg::DeviceState>("/autonav/state/device", 10, std::bind(&AutoNode::onDeviceState, this, std::placeholders::_1));
		deviceStateClient = this->create_client<autonav_msgs::srv::SetDeviceState>("/autonav/state/set_device_state");
		systemStateClient = this->create_client<autonav_msgs::srv::SetSystemState>("/autonav/state/set_system_state");

		// Configuration
		auto configurationSubscriber = this->create_subscription<autonav_msgs::msg::ConfigurationInstruction>("/autonav/configuration", 10, std::bind(&Configuration::onConfigurationInstruction, &config, std::placeholders::_1));
		auto configurationPublisher = this->create_publisher<autonav_msgs::msg::ConfigurationInstruction>("/autonav/configuration", 10);
		config = Configuration(id, configurationSubscriber, configurationPublisher);
	}

	AutoNode::~AutoNode()
	{
	}

	int64_t AutoNode::getDeviceID()
	{
		return id;
	}

	void AutoNode::setSystemState(SystemState state)
	{
		// Send a system state client request
		auto request = std::make_shared<autonav_msgs::srv::SetSystemState::Request>();
		request->state = static_cast<uint8_t>(state);
		systemStateClient->async_send_request(request);
	}

	void AutoNode::setDeviceState(DeviceState state)
	{
		auto request = std::make_shared<autonav_msgs::srv::SetDeviceState::Request>();
		request->device = getDeviceID();
		request->state = static_cast<uint8_t>(state);
		deviceStateClient->async_send_request(request);
	}

	void AutoNode::onSystemState(const autonav_msgs::msg::SystemState::SharedPtr msg)
	{	
		state = *msg;
	}

	void AutoNode::onDeviceState(const autonav_msgs::msg::DeviceState::SharedPtr msg)
	{
		if (msg->device != getDeviceID())
		{
			deviceStates[msg->device] = static_cast<DeviceState>(msg->state);
			return;
		}

		if (msg->state == static_cast<uint8_t>(DeviceState::STANDBY))
		{
			config.recache();
			configure();
			deviceStates[msg->device] = static_cast<DeviceState>(msg->state);
			return;
		}

		if (transition(static_cast<DeviceState>(msg->state)))
		{
			deviceStates[msg->device] = static_cast<DeviceState>(msg->state, true);
		}
	}

	bool AutoNode::transition(DeviceState state)
	{
		UNUSED(state);
		return true;
	}

	autonav_msgs::msg::SystemState AutoNode::getSystemState()
	{
		return state;
	}

	DeviceState AutoNode::getDeviceState()
	{
		return deviceStates[getDeviceID()];
	}

	DeviceState AutoNode::getDeviceState(std::string id)
	{
		return deviceStates[Autonav::hash(id)];
	}

	std::map<int64_t, DeviceState> AutoNode::getDeviceStates()
	{
		return deviceStates;
	}
}