#include "scr_msgs/srv/set_device_state.hpp"
#include "scr_msgs/srv/set_system_state.hpp"
#include "scr_msgs/msg/device_state.hpp"
#include "scr_msgs/msg/system_state.hpp"
#include "scr_core/system_state.h"
#include "scr_core/device_state.h"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

class StateSystemNode : public rclcpp::Node
{
public:
	StateSystemNode() : Node("scr_state_system")
	{
		systemStateService = this->create_service<scr_msgs::srv::SetSystemState>("/scr/state/set_system_state", std::bind(&StateSystemNode::onSetSystemState, this, std::placeholders::_1, std::placeholders::_2));
		deviceStateService = this->create_service<scr_msgs::srv::SetDeviceState>("/scr/state/set_device_state", std::bind(&StateSystemNode::onSetDeviceState, this, std::placeholders::_1, std::placeholders::_2));
		deviceStatePublisher = this->create_publisher<scr_msgs::msg::DeviceState>("/scr/state/device", 10);
		systemStatePublisher = this->create_publisher<scr_msgs::msg::SystemState>("/scr/state/system", 10);
		broadcastSubscriber = this->create_subscription<std_msgs::msg::Empty>("/scr/state/broadcast", 10, std::bind(&StateSystemNode::onBroadcast, this, std::placeholders::_1));

		stateTimer = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&StateSystemNode::onStateTick, this));

		requiredNodes = declare_parameter("required_nodes", std::vector<std::string>());
		if (std::find(requiredNodes.begin(), requiredNodes.end(), "scr_logging") == requiredNodes.end())
		{
			requiredNodes.push_back("scr_logging");
		}
		if (std::find(requiredNodes.begin(), requiredNodes.end(), "scr_configuration") == requiredNodes.end())
		{
			requiredNodes.push_back("scr_configuration");
		}

		state = scr_msgs::msg::SystemState();
		state.state = SCR::SystemState::DISABLED;
		state.mode = declare_parameter("mode", 0);
		state.estop = false;
		state.mobility = declare_parameter("override_mobility", false);

		auto forcedState = declare_parameter("forced_state", "");
		if (forcedState == "disabled")
		{
			state.state = SCR::SystemState::DISABLED;
		}
		else if (forcedState == "autonomous")
		{
			state.state = SCR::SystemState::AUTONOMOUS;
		}
		else if (forcedState == "manual")
		{
			state.state = SCR::SystemState::MANUAL;
		}
		
		else if (forcedState == "shutdown")
		{
			state.state = SCR::SystemState::SHUTDOWN;
		}
	}

	void onBroadcast(const std_msgs::msg::Empty::SharedPtr _)
	{
		systemStatePublisher->publish(state);
		for (auto &[id, state] : deviceStates)
		{
			auto deviceState = scr_msgs::msg::DeviceState();
			deviceState.device = id;
			deviceState.state = state;
			deviceStatePublisher->publish(deviceState);
		}
	}

	bool shouldIgnoreNode(std::string node)
	{
		if (node.find("scr_state_system") != std::string::npos)
		{
			return true;
		}

		if (node.find("scr_configuration") != std::string::npos)
		{
			return false;
		}

		if (node.find("scr_logging") != std::string::npos)
		{
			return false;
		}	

		if (node.find("autonav") != std::string::npos)
		{
			return false;
		}

		return true;
	}

	void onStateTick()
	{
		if (state.state == SCR::SystemState::SHUTDOWN)
		{
			kill(getpid(), SIGINT);
			return;
		}

		auto rawNodes = this->get_node_names();
		std::vector<std::string> nodes;
		for (auto node : rawNodes)
		{
			if (node.length() == 0 || node[0] != '/')
			{
				continue;
			}
			nodes.push_back(node.substr(1));
		}

		for (auto node : nodes)
		{
			if(shouldIgnoreNode(node))
			{
				continue;
			}

			if (std::find(trackedNodes.begin(), trackedNodes.end(), node) == trackedNodes.end())
			{
				trackedNodes.push_back(node);
				onNodeAdded(node);
				publishState();
			}
		}

		std::vector<std::string> lostNodes;
		for (auto node : trackedNodes)
		{
			if (std::find(nodes.begin(), nodes.end(), node) == nodes.end())
			{
				lostNodes.push_back(node);
			}
		}

		for (auto node : lostNodes)
		{
			trackedNodes.erase(std::remove(trackedNodes.begin(), trackedNodes.end(), node), trackedNodes.end());
			onNodeRemoved(node);
		}

		if (waitingQueue.size() > 0 && areRequiredNodesRunning())
		{
			emptyWaitingQueue();
		}
	}

	void publishState()
	{
		systemStatePublisher->publish(state);
	}
	
	bool areRequiredNodesRunning()
	{
		for (auto node : requiredNodes)
		{
			if (std::find(trackedNodes.begin(), trackedNodes.end(), node) == trackedNodes.end())
			{
				return false;
			}
		}

		return true;
	}

	void emptyWaitingQueue()
	{
		for (auto node : waitingQueue)
		{
			auto deviceState = scr_msgs::msg::DeviceState();
			deviceState.device = node;
			deviceState.state = SCR::DeviceState::STANDBY;
			deviceStatePublisher->publish(deviceState);
			deviceStates[node] = SCR::DeviceState::STANDBY;
		}

		waitingQueue.clear();
	}

	void onNodeAdded(const std::string& node)
	{
		// Publish a new device state of standby for them
		if (!areRequiredNodesRunning())
		{
			waitingQueue.push_back(node);
			return;
		}

		auto deviceState = scr_msgs::msg::DeviceState();
		deviceState.device = node;
		deviceState.state = SCR::DeviceState::STANDBY;
		deviceStatePublisher->publish(deviceState);
		deviceStates[node] = SCR::DeviceState::STANDBY;
		publishState();
	}

	void onNodeRemoved(const std::string& node)
	{
		// Publish a new device state of standby for them
		auto deviceState = scr_msgs::msg::DeviceState();
		deviceState.device = node;
		deviceState.state = SCR::DeviceState::OFF;
		deviceStatePublisher->publish(deviceState);

		auto wasRequired = std::find(requiredNodes.begin(), requiredNodes.end(), node) != requiredNodes.end();
		if (wasRequired)
		{
			state.state = SCR::SystemState::SHUTDOWN;
			publishState();
		}
	}

	void onSetSystemState(const std::shared_ptr<scr_msgs::srv::SetSystemState::Request> request, std::shared_ptr<scr_msgs::srv::SetSystemState::Response> response)
	{
		state.state = request->state;
		state.estop = request->estop;
		state.mobility = request->mobility;
		publishState();

		response->ok = true;
	}

	void onSetDeviceState(const std::shared_ptr<scr_msgs::srv::SetDeviceState::Request> request, std::shared_ptr<scr_msgs::srv::SetDeviceState::Response> response)
	{
		if (deviceStates[request->device] == request->state)
		{
			response->ok = true;
			return;
		}

		auto deviceState = scr_msgs::msg::DeviceState();
		deviceState.device = request->device;
		deviceState.state = request->state;
		deviceStatePublisher->publish(deviceState);
		deviceStates[request->device] = request->state;
		response->ok = true;
		publishState();
	}

private:
	std::vector<std::string> requiredNodes;
	std::vector<std::string> trackedNodes;
	std::vector<std::string> waitingQueue;
	std::map<std::string, uint8_t> deviceStates;
	scr_msgs::msg::SystemState state;

	std::shared_ptr<rclcpp::Publisher<scr_msgs::msg::DeviceState>> deviceStatePublisher;
	std::shared_ptr<rclcpp::Publisher<scr_msgs::msg::SystemState>> systemStatePublisher;
	std::shared_ptr<rclcpp::Service<scr_msgs::srv::SetSystemState>> systemStateService;
	std::shared_ptr<rclcpp::Service<scr_msgs::srv::SetDeviceState>> deviceStateService;
	std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Empty>> broadcastSubscriber;
	std::shared_ptr<rclcpp::TimerBase> stateTimer;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<StateSystemNode>());
	rclcpp::shutdown();
}