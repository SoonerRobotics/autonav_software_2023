#pragma once

#include "scr_msgs/msg/configuration_instruction.hpp"
#include "scr_msgs/msg/performance_result.hpp"
#include "scr_msgs/srv/set_system_state.hpp"
#include "scr_msgs/srv/set_device_state.hpp"
#include "scr_msgs/msg/device_state.hpp"
#include "scr_msgs/msg/system_state.hpp"
#include "std_msgs/msg/empty.hpp"
#include "scr_msgs/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "configuration.h"
#include "performance.h"
#include "device_state.h"
#include "system_state.h"
#include "system_mode.h"
#include <stdint.h>
#include <string.h>
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
		void setSystemMode(SystemMode mode);
		void setMobility(bool state);
		void setEStop(bool state);

		Configuration config;
		Performance performance;

		DeviceState getDeviceState();
		DeviceState getDeviceState(std::string device);
		scr_msgs::msg::SystemState getSystemState();

		std::map<std::string, DeviceState> getDeviceStates();

		void log(const std::string& message);
		void reset();

	protected:
		virtual void configure();
		virtual void onReset();
		virtual void transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated);
		virtual void onDeviceStateUpdated(const scr_msgs::msg::DeviceState::SharedPtr msg, bool isNew);

	private:
		void onResetInternal(const std_msgs::msg::Empty::SharedPtr msg);
		void onSystemState(const scr_msgs::msg::SystemState::SharedPtr msg);
		void onDeviceState(const scr_msgs::msg::DeviceState::SharedPtr msg);

	private:
		rclcpp::Subscription<scr_msgs::msg::SystemState>::SharedPtr systemStateSubscriber;
		rclcpp::Subscription<scr_msgs::msg::DeviceState>::SharedPtr deviceStateSubscriber;
		rclcpp::Client<scr_msgs::srv::SetSystemState>::SharedPtr systemStateClient;
		rclcpp::Client<scr_msgs::srv::SetDeviceState>::SharedPtr deviceStateClient;
		rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resetSubscriber;
		rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr resetPublisher;
		rclcpp::Publisher<scr_msgs::msg::Log>::SharedPtr logPublisher;
		std::map<std::string, DeviceState> deviceStates;
		scr_msgs::msg::SystemState state;
	};
}