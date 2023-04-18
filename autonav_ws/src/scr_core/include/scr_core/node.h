#pragma once

#include "autonav_msgs/msg/configuration_instruction.hpp"
#include "autonav_msgs/srv/set_system_state.hpp"
#include "autonav_msgs/srv/set_device_state.hpp"
#include "autonav_msgs/msg/device_state.hpp"
#include "autonav_msgs/msg/system_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "configuration.h"
#include "device_state.h"
#include "system_state.h"
#include <stdint.h>
#include <string.h>
#include "utils.h"
#include <map>

namespace SCR
{
	class Node : public rclcpp::Node
	{
	public:
		Node(std::string node_name);
		~Node();

		void setSystemState(SystemState state);
		void setDeviceState(DeviceState state);
		void setEStop(bool state);
		void setMobility(bool state);

		Configuration config;

		DeviceState getDeviceState();
		DeviceState getDeviceState(std::string device);
		autonav_msgs::msg::SystemState getSystemState();

		int64_t getDeviceID();
		std::map<int64_t, DeviceState> getDeviceStates();

	protected:
		virtual void configure();
		virtual void transition(autonav_msgs::msg::SystemState old, autonav_msgs::msg::SystemState updated);

	private:
		void onSystemState(const autonav_msgs::msg::SystemState::SharedPtr msg);
		void onDeviceState(const autonav_msgs::msg::DeviceState::SharedPtr msg);

	private:
		rclcpp::Subscription<autonav_msgs::msg::SystemState>::SharedPtr systemStateSubscriber;
		rclcpp::Subscription<autonav_msgs::msg::DeviceState>::SharedPtr deviceStateSubscriber;
		rclcpp::Client<autonav_msgs::srv::SetSystemState>::SharedPtr systemStateClient;
		rclcpp::Client<autonav_msgs::srv::SetDeviceState>::SharedPtr deviceStateClient;
		std::map<int64_t, DeviceState> deviceStates;
		autonav_msgs::msg::SystemState state;

		int64_t id;
	};
}