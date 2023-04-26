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

	Configuration::Configuration(int64_t id, rclcpp::Subscription<scr_msgs::msg::ConfigurationInstruction>::SharedPtr configSubscriber, rclcpp::Publisher<scr_msgs::msg::ConfigurationInstruction>::SharedPtr configPublisher)
	{
		this->id = id;
		this->configSubscriber = configSubscriber;
		this->configPublisher = configPublisher;
		loadLocalPresets();
	}

	Configuration::~Configuration()
	{
	}

	bool Configuration::hasLoadedPreset()
	{
		return preset != "";
	}

	bool Configuration::hasDevice(int64_t device)
	{
		return cache.find(device) != cache.end();
	}

	void Configuration::loadLocalPresets()
	{
		// Load all presets from the local configuration directory into the presets vector, removing the file extension
		std::string path = "/home/" + std::string(getenv("USER")) + "/.scr/configuration";
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
			std::string path = "/home/" + std::string(getenv("USER")) + "/.scr/configuration/" + preset + ".json";
			std::filesystem::remove(path);
			presets.erase(std::remove(presets.begin(), presets.end(), preset), presets.end());
			load("default");
		}
	}

	void Configuration::deleteAllPresets()
	{
		std::string path = "/home/" + std::string(getenv("USER")) + "/.scr/configuration";
		std::filesystem::remove_all(path);
		presets.clear();
		load("default");
	}

	void Configuration::save(const std::string& preset)
	{
		std::string configPath = "/home/" + std::string(getenv("USER")) + "/.scr/configuration";
		std::string path = configPath + "/" + preset + ".json";
		std::ofstream file(path);

		RCLCPP_INFO(rclcpp::get_logger("scr_core"), "Saving configuration file: %s", path.c_str());

		if (!std::filesystem::exists(configPath))
		{
			RCLCPP_INFO(rclcpp::get_logger("scr_core"), "Creating configuration directory: %s", configPath.c_str());
			std::filesystem::create_directories(configPath);
		}

		for (auto device : cache)
		{
			for (auto address : device.second)
			{
				file << device.first << "," << (int)address.first << ",";
				for (auto byte : address.second)
				{
					file << (int)byte << ":";
				}
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
		std::string path = "/home/" + std::string(getenv("USER")) + "/.scr/configuration/" + preset + ".json";
		if (!std::filesystem::exists(path))
		{
			if (preset != "default")
			{
				RCLCPP_INFO(rclcpp::get_logger("scr_core"), "Configuration file not found. Loading default configuration.");
				load("default");
				return;
			}
			else
			{
				RCLCPP_INFO(rclcpp::get_logger("scr_core"), "No configuration file found. Creating default configuration.");
				this->preset = "default";
				save("default");
				return;
			}	
		}

		RCUTILS_LOG_INFO("Loading configuration file: %s", path.c_str());
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

			int64_t device = std::stoll(tokens[0]);
			uint8_t address = std::stoi(tokens[1]);

			std::vector<uint8_t> bytes;
			std::istringstream byteStream(tokens[2]);
			while (std::getline(byteStream, token, ':'))
			{
				bytes.push_back(std::stoi(token));
			}

			scr_msgs::msg::ConfigurationInstruction instruction;
			instruction.device = device;
			instruction.address = address;
			instruction.data = bytes;
			configPublisher->publish(instruction);
		}

		this->preset = preset;
		file.close();
	}

	std::vector<std::string> Configuration::getPresets()
	{
		return presets;
	}

	template <>
	int Configuration::get(uint8_t address)
	{
		return *(int*)cache[id][address].data();
	}

	template <>
	int Configuration::get(int64_t device, uint8_t address)
	{
		auto bytes = cache[device][address];
		return static_cast<int>(bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3]);
	}

	template <>
	float Configuration::get(uint8_t address)
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
	float Configuration::get(int64_t device, uint8_t address)
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
	void Configuration::set(uint8_t address, int value)
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
	void Configuration::set(int64_t device, uint8_t address, int value)
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
	void Configuration::set(uint8_t address, float value)
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
	void Configuration::set(int64_t device, uint8_t address, float value)
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
	bool Configuration::get(uint8_t address)
	{
		return cache[id][address][0] == 1;
	}

	template<>
	bool Configuration::get(int64_t device, uint8_t address)
	{
		return cache[device][address][0] == 1;
	}

	template<>
	void Configuration::set(uint8_t address, bool value)
	{
		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = id;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = std::vector<uint8_t>{(uint8_t)value};
		configPublisher->publish(instruction);
	}

	template<>
	void Configuration::set(int64_t device, uint8_t address, bool value)
	{
		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = device;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = std::vector<uint8_t>{(uint8_t)value};
		configPublisher->publish(instruction);
	}

	void Configuration::recache()
	{
		cache.clear();

		scr_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = id;
		instruction.opcode = Opcode::GET_ALL;
		configPublisher->publish(instruction);
	}

	std::map<int64_t, std::map<uint8_t, std::vector<uint8_t>>> Configuration::getCache()
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
				cache[instruction->device] = std::map<uint8_t, std::vector<uint8_t>>();
			}
			cache[instruction->device][instruction->address] = instruction->data;
		}

		if (instruction->opcode == Opcode::GET_ALL && !amTarget)
		{
			auto registers = cache[this->id];
			for (auto &[address, bits] : registers)
			{
				scr_msgs::msg::ConfigurationInstruction response;
				response.device = instruction->device;
				response.opcode = Opcode::GET_ACK;
				response.address = address;
				response.data = bits;
				configPublisher->publish(response);
			}
		}
	}
}