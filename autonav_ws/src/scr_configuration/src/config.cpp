#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "scr_core/node.h"
#include "scr_msgs/msg/log.hpp"
#include <filesystem>

bool doesFileExist(const std::string &path)
{
	return std::filesystem::exists(path);
}

void createPath(const std::string &path)
{
	std::filesystem::create_directories(path);
}

std::string path(const std::string &path)
{
	std::string user = std::string(getenv("USER"));
	std::string result = path;
	size_t pos = result.find("{user}");
	result.replace(pos, 6, user);
	return result;
}

static const std::string CONFIG_PATH = "/home/{user}/.scr/configuration/";

class ConfigurationNode : public SCR::Node
{
public:
	ConfigurationNode() : SCR::Node("scr_configuration")
	{
		loadSubscription = create_subscription<std_msgs::msg::String>("/scr/configuration/load", 20, std::bind(&ConfigurationNode::onLoadInstruction, this, std::placeholders::_1));
		presetPublisher = create_publisher<std_msgs::msg::String>("/scr/configuration/preset", 20);
		preset = declare_parameter("preset", "default");
	}

	void onLoadInstruction(std_msgs::msg::String::SharedPtr msg)
	{
		if (msg->data == preset)
		{
			return;
		}

		if (doesFileExist(path(CONFIG_PATH) + msg->data + ".csv"))
		{
			loadPreset(path(CONFIG_PATH) + msg->data + ".csv");
		}
		else
		{
			savePreset(msg->data);
		}
	}

	void loadPreset(const std::string &path)
	{
		if (!doesFileExist(path))
		{
			return;
		}

		std::ifstream file(path);
		std::string line;

		while (getline(file, line))
		{
			std::vector<std::string> tokens;
			std::string token;
			std::istringstream tokenStream(line);
			while (std::getline(tokenStream, token, ','))
			{
				tokens.push_back(token);
			}

			std::string device = tokens[0];
			std::string address = tokens[1];

			std::vector<uint8_t> bytes;
			std::istringstream byteStream(tokens[2]);
			while (std::getline(byteStream, token, ':'))
			{
				bytes.push_back(std::stoi(token));
			}

			temporaryCache[device][address] = bytes;
		}

		// Loop through all devices we are aware of and send relevant configuration
		for (auto &[deviceId, deviceState] : this->getDeviceStates())
		{
			if (deviceState == SCR::DeviceState::OFF)
			{
				continue;
			}

			if (temporaryCache.find(deviceId) == temporaryCache.end())
			{
				continue;
			}

			for (auto &[address, bytes] : temporaryCache.at(deviceId))
			{
				config.set(deviceId, address, bytes);
			}
		}

		preset = path.substr(path.find_last_of("/") + 1);
		preset = preset.substr(0, preset.find_last_of("."));
		auto presetMsg = std_msgs::msg::String();
		presetMsg.data = preset;
		presetPublisher->publish(presetMsg);
	}

	void savePreset(const std::string &preset)
	{
		this->config.save(preset);
	}

	void onDeviceStateUpdated(const scr_msgs::msg::DeviceState::SharedPtr msg, bool isNew) override
	{
		if (!isNew || msg->state == SCR::DeviceState::OFF)
		{
			return;
		}

		if (temporaryCache.find(msg->device) != temporaryCache.end())
		{
			for (auto &[address, bytes] : temporaryCache.at(msg->device))
			{
				config.set(msg->device, address, bytes);
			}
		}

		if (temporaryCache.find(msg->device) == temporaryCache.end())
		{
			return;
		}

		for (auto &reg : temporaryCache.at(msg->device))
		{
			config.set(msg->device, reg.first, reg.second);
		}

		auto presetMsg = std_msgs::msg::String();
		presetMsg.data = preset;
		presetPublisher->publish(presetMsg);
	}

	void configure() override
	{
		createPath(path(CONFIG_PATH));
		if (doesFileExist(path(CONFIG_PATH) + preset + ".csv"))
		{
			// loadPreset(path(CONFIG_PATH) + preset + ".csv");
		}
		else
		{
			// savePreset(preset);
		}

		setDeviceState(SCR::DeviceState::OPERATING);
	}

	void transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override
	{
	}

private:
	std::string preset;
	std::map<std::string, std::map<std::string, std::vector<uint8_t>>> temporaryCache;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr loadSubscription;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr presetPublisher;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ConfigurationNode>());
	rclcpp::shutdown();
	return 0;
}