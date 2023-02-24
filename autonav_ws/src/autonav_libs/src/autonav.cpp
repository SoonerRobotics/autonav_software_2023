#include "autonav_libs/autonav.h"

namespace Autonav
{
	namespace ROS
	{
		AutoNode::AutoNode(Autonav::Device device, std::string node_name) : Node(node_name)
		{
			_device = device;
			_systemState = State::SystemState::DISABLED;
			_deviceState = State::DeviceState::OFF;

			_systemStatePublisher = this->create_publisher<autonav_msgs::msg::SystemState>("/autonav/state/system", 10);
			_deviceStatePublisher = this->create_publisher<autonav_msgs::msg::DeviceState>("/autonav/state/device", 10);
			_systemStateSubscriber = this->create_subscription<autonav_msgs::msg::SystemState>("/autonav/state/system", 10, std::bind(&AutoNode::onSystemState, this, std::placeholders::_1));
			_deviceStateSubscriber = this->create_subscription<autonav_msgs::msg::DeviceState>("/autonav/state/device", 10, std::bind(&AutoNode::onDeviceState, this, std::placeholders::_1));
		
			setDeviceState(State::DeviceState::OFF);
		}

		AutoNode::~AutoNode()
		{
		}

		void AutoNode::setSystemState(State::SystemState state)
		{
			_systemState = state;
			auto msg = autonav_msgs::msg::SystemState();
			msg.state = static_cast<uint8_t>(_systemState);
			_systemStatePublisher->publish(msg);
		}

		void AutoNode::setDeviceState(State::DeviceState state)
		{
			_deviceState = state;
			auto msg = autonav_msgs::msg::DeviceState();
			msg.device = static_cast<uint8_t>(_device);
			msg.state = static_cast<uint8_t>(_deviceState);
			_deviceStatePublisher->publish(msg);
		
			if (msg.device >= _deviceStates.size())
			{
				_deviceStates.resize(msg.device + 1);
			}
			_deviceStates[msg.device] = static_cast<State::DeviceState>(msg.state);
		}

		void AutoNode::onSystemState(const autonav_msgs::msg::SystemState::SharedPtr msg)
		{
			_systemState = static_cast<State::SystemState>(msg->state);
		}

		void AutoNode::onDeviceState(const autonav_msgs::msg::DeviceState::SharedPtr msg)
		{
			if (msg->device == static_cast<uint8_t>(_device))
			{
				// Do we want this to possible? Is there any reason for this to happen?
				_deviceState = static_cast<State::DeviceState>(msg->state);
			}

			if (msg->device >= _deviceStates.size())
			{
				_deviceStates.resize(msg->device + 1);
			}
			_deviceStates[msg->device] = static_cast<State::DeviceState>(msg->state);
		}
	}
}