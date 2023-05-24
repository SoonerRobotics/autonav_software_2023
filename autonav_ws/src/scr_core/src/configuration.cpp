#include "scr_msgs/msg/configuration_instruction.hpp"
#include "scr_core/configuration.h"
#include "rclcpp/rclcpp.hpp"
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>

namespace SCR
{
	Configuration::Configuration()
	{
		loadLocalPresets();
	}

	Configuration::Configuration(
		std::string id, 
		rclcpp::Subscription<scr_msgs::msg::ConfigurationInstruction>::SharedPtr configSubscriber, 
		rclcpp::Publisher<scr_msgs::msg::ConfigurationInstruction>::SharedPtr configPublisher, 
		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr loadSubscription,
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr loadPublisher
	)
	{
		this->id = id;
		this->configSubscriber = configSubscriber;
		this->configPublisher = configPublisher;
		this->loadSubscription = loadSubscription;
		this->loadPublisher = loadPublisher;
		this->preset = "";
		this->presets = {};

		loadLocalPresets();
	}

	Configuration::~Configuration()
	{
	}

	bool Configuration::hasLoadedPreset()
	{
		return preset != "";
	}

	bool Configuration::hasDevice(std::string device)
	{
		return cache.find(device) != cache.end();
	}

	void Configuration::loadLocalPresets()
	{
		// Load all presets from the local configuration directory into the presets vector, removing the file extension
		std::string path = "/home/" + std::string(getenv("USER")) + "/.scr/configuration/";

		if (!std::filesystem::exists(path))
		{
			RCLCPP_INFO(rclcpp::get_logger("scr_core"), "Creating configuration directory: %s", path.c_str());
			std::filesystem::create_directories(path);
		}

		for (const auto &entry : std::filesystem::directory_iterator(path))
		{
			std::string preset = entry.path().filename().string();
			preset = preset.substr(0, preset.find_last_of("."));
			presets.push_back(preset);
		}
	}

	std::string Configuration::getActivePreset()
	{
		return preset;
	}

	void Configuration::deleteActivePreset()
	{
		if (preset != "")
		{
			std::string path = "/home/" + std::string(getenv("USER")) + "/.scr/configuration/" + preset + ".csv";
			std::filesystem::remove(path);
			presets.erase(std::remove(presets.begin(), presets.end(), preset), presets.end());
			load("default");
		}
	}

	void Configuration::deleteAllPresets()
	{
		std::string path = "/home/" + std::string(getenv("USER")) + "/.scr/configuration/";
		std::filesystem::remove_all(path);
		presets.clear();
		load("default");
	}

	void Configuration::save(const std::string& preset)
	{
		std::string configPath = "/home/" + std::string(getenv("USER")) + "/.scr/configuration/";
		std::string path = configPath + preset + ".csv";

		RCLCPP_INFO(rclcpp::get_logger("scr_core"), "Saving configuration file: %s", path.c_str());

		if (!std::filesystem::exists(configPath))
		{
			RCLCPP_INFO(rclcpp::get_logger("scr_core"), "Creating configuration directory: %s", configPath.c_str());
			std::filesystem::create_directories(configPath);
		}

		std::ofstream file(path);
		for (auto device : cache)
		{
			for (auto address : device.second)
			{
				file << device.first << "," << address.first << ",";
				std::string byteStr = "";
				for (auto byte : address.second)
				{
					byteStr += std::to_string(byte) + ":";
				}

				byteStr = byteStr.substr(0, byteStr.size() - 1);
				file << byteStr;
				file << std::endl;
			}
		}

		file.close();
		if (std::find(presets.begin(), presets.end(), preset) == presets.end())
		{
			presets.push_back(preset);
		}
	}

	void Configuration::load(const std::string& preset)
	{
		std_msgs::msg::String msg;
		msg.data = preset;
		loadPublisher->publish(msg);
	}

	void Configuration::onPresetChanged(std_msgs::msg::String::SharedPtr msg)
	{
		this->preset = msg->data;
	}

	std::vector<std::string> Configuration::getPresets()
	{
		return presets;
	}

	template <>
	int Configuration::get(std::string address)
	{
		return *(int*)cache[id][address].data();
	}

	template <>
	int Configuration::get(std::string device, std::string address)
	{
		auto bytes = cache[device][address];
		return static_cast<int>(bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3]);
	}

	bool Configuration::has(std::string device, std::string address)
	{
		return cache[device].find(address) != cache[device].end();
	}

	template <>
	float Configuration::get(std::string address)
	{
		unsigned char byte_array[] = {cache[id][address][0], cache[id][address][1], cache[id][address][2], cache[id][address][3]};
		float value;
		std::copy(
			reinterpret_cast<const char*>(&byte_array[0]),
			reinterpret_cast<const char*>(&byte_array[4]),
			reinterpret_cast<char*>(&value)
		);
		return value;
	}

	template <>
	float Configuration::get(std::string device, std::string address)
	{
		auto bytes = cache[device][address];
		unsigned char byte_array[] = {bytes[0], bytes[1], bytes[2], bytes[3]};
		float value;
		std::copy(
			reinterpret_cast<const char*>(&byte_array[0]),
			reinterpret_cast<const char*>(&byte_array[4]),
			reinterpret_cast<char*>(&value)
		);
		return value;
	}

	template <>
	void Configuration::set(std::string address, int value)
	{
		std::vector<uint8_t> bytes;
		bytes.push_back((value >> 24) & 0xFF);
		bytes.push_back((value >> 16) & 0xFF);
		bytes.push_back((value >> 8) & 0xFF);
		bytes.push_back(value & 0xFF);

		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = id;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = bytes;
		configPublisher->publish(instruction);
	}

	template <>
	void Configuration::set(std::string device, std::string address, int value)
	{
		std::vector<uint8_t> bytes;
		bytes.push_back((value >> 24) & 0xFF);
		bytes.push_back((value >> 16) & 0xFF);
		bytes.push_back((value >> 8) & 0xFF);
		bytes.push_back(value & 0xFF);

		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = device;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = bytes;
		configPublisher->publish(instruction);
	}

	template<>
	void Configuration::set(std::string address, float value)
	{
		unsigned const char *p = reinterpret_cast<unsigned const char *>(&value);
		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = id;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = std::vector<uint8_t>(p, p + sizeof(float));
		configPublisher->publish(instruction);
	}

	template<>
	void Configuration::set(std::string device, std::string address, float value)
	{
		unsigned const char *p = reinterpret_cast<unsigned const char *>(&value);
		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = device;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = std::vector<uint8_t>(p, p + sizeof(float));
		configPublisher->publish(instruction);
	}

	template<>
	bool Configuration::get(std::string address)
	{
		return cache[id][address][0] == 1;
	}

	template<>
	bool Configuration::get(std::string device, std::string address)
	{
		return cache[device][address][0] == 1;
	}

	template<>
	void Configuration::set(std::string address, bool value)
	{
		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = id;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = std::vector<uint8_t>{(uint8_t)value};
		configPublisher->publish(instruction);
	}

	template<>
	void Configuration::set(std::string device, std::string address, bool value)
	{
		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = device;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = std::vector<uint8_t>{(uint8_t)value};
		configPublisher->publish(instruction);
	}

	template<>
	void Configuration::set(std::string address, std::vector<uint8_t> bytes)
	{
		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = id;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = bytes;
		configPublisher->publish(instruction);
	}

	template<>
	void Configuration::set(std::string device, std::string address, std::vector<uint8_t> bytes)
	{
		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = device;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = bytes;
		configPublisher->publish(instruction);
	}

	void Configuration::recache()
	{
		cache.clear();

		// For each device on the network, send a GET_ALL request
	}

	std::map<std::string, std::map<std::string, std::vector<uint8_t>>> Configuration::getCache()
	{
		return cache;
	}

	void Configuration::onConfigurationInstruction(const scr_msgs::msg::ConfigurationInstruction::SharedPtr instruction)
	{
		const bool amTarget = instruction->device == id;

		if (instruction->opcode == Opcode::GET && amTarget)
		{
			scr_msgs::msg::ConfigurationInstruction response;
			response.device = instruction->device;
			response.opcode = Opcode::GET_ACK;
			response.address = instruction->address;
			response.data = cache[instruction->device][instruction->address];
			configPublisher->publish(response);
		}

		if (instruction->opcode == Opcode::SET && amTarget)
		{
			cache[instruction->device][instruction->address] = instruction->data;
			scr_msgs::msg::ConfigurationInstruction response;
			response.device = instruction->device;
			response.opcode = Opcode::SET_ACK;
			response.address = instruction->address;
			response.data = instruction->data;
			configPublisher->publish(response);
		}

		if (instruction->opcode == Opcode::SET_ACK || instruction->opcode == Opcode::GET_ACK)
		{
			// if cache[instruction->device] doesn't exist, it will be created
			if (cache.find(instruction->device) == cache.end())
			{
				cache[instruction->device] = std::map<std::string, std::vector<uint8_t>>();
			}
			cache[instruction->device][instruction->address] = instruction->data;
		}

		if (instruction->opcode == Opcode::GET_ALL && amTarget)
		{
			for (auto const& [address, bytes] : cache[instruction->device])
			{
				scr_msgs::msg::ConfigurationInstruction response;
				response.device = instruction->device;
				response.opcode = Opcode::GET_ACK;
				response.address = address;
				response.data = bytes;
				configPublisher->publish(response);
			}
		}
	}
}