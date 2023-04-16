#include "autonav_msgs/msg/configuration_instruction.hpp"
#include "autonav_libs/configuration.h"
#include "rclcpp/rclcpp.hpp"
#include <unistd.h>

namespace Autonav
{
	Configuration::Configuration()
	{
	}

	Configuration::Configuration(int64_t id, rclcpp::Subscription<autonav_msgs::msg::ConfigurationInstruction>::SharedPtr configSubscriber, rclcpp::Publisher<autonav_msgs::msg::ConfigurationInstruction>::SharedPtr configPublisher)
	{
		this->id = id;
		this->configSubscriber = configSubscriber;
		this->configPublisher = configPublisher;
	}

	Configuration::~Configuration()
	{
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
	void Configuration::set(uint8_t address, int value)
	{
		std::vector<uint8_t> bytes;
		bytes.push_back((value >> 24) & 0xFF);
		bytes.push_back((value >> 16) & 0xFF);
		bytes.push_back((value >> 8) & 0xFF);
		bytes.push_back(value & 0xFF);

		autonav_msgs::msg::ConfigurationInstruction instruction;
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

		autonav_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = device;
		instruction.opcode = Opcode::SET;
		instruction.address = address;
		instruction.data = bytes;
		configPublisher->publish(instruction);
	}

	void Configuration::recache()
	{
		cache.clear();

		autonav_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = id;
		instruction.opcode = Opcode::GET_ALL;
		configPublisher->publish(instruction);
	}

	std::map<int64_t, std::map<uint8_t, std::vector<uint8_t>>> Configuration::getCache()
	{
		return cache;
	}

	void Configuration::onConfigurationInstruction(const autonav_msgs::msg::ConfigurationInstruction::SharedPtr instruction)
	{
		const bool amTarget = instruction->device == id;

		RCLCPP_INFO(rclcpp::get_logger("Configuration"), "Received configuration instruction: %d %d %d", instruction->device, instruction->opcode, instruction->address);

		if (instruction->opcode == Opcode::GET && amTarget)
		{
			autonav_msgs::msg::ConfigurationInstruction response;
			response.device = instruction->device;
			response.opcode = Opcode::GET_ACK;
			response.address = instruction->address;
			response.data = cache[instruction->device][instruction->address];
			configPublisher->publish(response);
		}

		if (instruction->opcode == Opcode::SET && amTarget)
		{
			cache[instruction->device][instruction->address] = instruction->data;
			autonav_msgs::msg::ConfigurationInstruction response;
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
				autonav_msgs::msg::ConfigurationInstruction response;
				response.device = instruction->device;
				response.opcode = Opcode::GET_ACK;
				response.address = address;
				response.data = bits;
				configPublisher->publish(response);
			}
		}
	}
}