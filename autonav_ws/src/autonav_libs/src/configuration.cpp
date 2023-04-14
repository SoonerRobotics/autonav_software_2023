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

	template <typename T>
	T Configuration::get(uint8_t address)
	{
		return get<T>(id, address);
	}

	template <typename T>
	T Configuration::get(uint8_t address, T default_value)
	{
		return get<T>(id, address, default_value);
	}

	template <typename T>
	T Configuration::get(int64_t device, uint8_t address)
	{
		return get<T>(device, address, T());
	}

	template <typename T>
	T Configuration::get(int64_t device, uint8_t address, T default_value)
	{
		if (cache.find(device) == cache.end())
		{
			return default_value;
		}

		if (cache[device].find(address) == cache[device].end())
		{
			return default_value;
		}

		return *(T*)cache[device][address].data();
	}

	template <typename T>
	void Configuration::set(uint8_t address, T value)
	{
		set<T>(id, address, value);
	}

	template <typename T>
	void Configuration::set(int64_t device, uint8_t address, T value)
	{
		std::vector<uint8_t> data(sizeof(T));
		memcpy(data.data(), &value, sizeof(T));
	}

	void Configuration::recache()
	{
		cache.clear();

		autonav_msgs::msg::ConfigurationInstruction instruction;
		instruction.device = id;
		instruction.opcode = Opcode::GET_ALL;
		configPublisher->publish(instruction);
	}

	void Configuration::onConfigurationInstruction(const autonav_msgs::msg::ConfigurationInstruction::SharedPtr instruction)
	{
		const bool amTarget = instruction->device == id;

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
			cache[instruction->device][instruction->address] = instruction->data;
		}

		if (instruction->opcode == Opcode::GET_ALL && !amTarget)
		{
			for (auto device : cache)
			{
				for (auto address : device.second)
				{
					autonav_msgs::msg::ConfigurationInstruction response;
					response.device = instruction->device;
					response.opcode = Opcode::GET_ACK;
					response.address = address.first;
					response.data = address.second;
					configPublisher->publish(response);
				}
			}
		}
	}
}