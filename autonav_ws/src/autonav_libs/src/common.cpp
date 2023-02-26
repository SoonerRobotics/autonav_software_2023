#include "autonav_libs/common.h"

// cout includes
#include <iostream>

namespace Autonav
{
	namespace ROS
	{
		AutoNode::AutoNode(Autonav::Device device, std::string node_name)
			: Node(node_name), _config(device, this)
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
			_deviceStates[_device] = _deviceState;
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

			_deviceStates[static_cast<Autonav::Device>(msg->device)] = static_cast<State::DeviceState>(msg->state);
		}
	}

	namespace Configuration
	{
		Conbus::Conbus(Device device, rclcpp::Node *node)
		{
			_device = device;

			_conbusSubscriber = node->create_subscription<autonav_msgs::msg::ConBusInstruction>("/autonav/conbus", 10, std::bind(&Conbus::onConbusInstruction, this, std::placeholders::_1));
			_conbusPublisher = node->create_publisher<autonav_msgs::msg::ConBusInstruction>("/autonav/conbus", 10);
		}

		Conbus::~Conbus()
		{
		}

		// TODO: Tell everyone else about our config changes!
		void Conbus::writeBytes(uint8_t address, std::vector<uint8_t> data)
		{
			_registers[_device][address] = data;

			auto msg = autonav_msgs::msg::ConBusInstruction();
			msg.device = static_cast<uint8_t>(_device);
			msg.address = address;
			msg.data = data;
			msg.opcode = ConbusOpcode::READ_ACK;
			_conbusPublisher->publish(msg);
		}


		void Conbus::write(uint8_t address, int32_t data)
		{
			writeBytes(address, {AddressType::INTEGER, 0, static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::write(uint8_t address, float data)
		{
			auto dataInt = static_cast<int32_t>(data * FLOAT_PRECISION);
			writeBytes(address, {AddressType::FLOAT, 0, static_cast<uint8_t>(dataInt >> 24), static_cast<uint8_t>(dataInt >> 16), static_cast<uint8_t>(dataInt >> 8), static_cast<uint8_t>(dataInt & 0xFF)});
		}

		void Conbus::write(uint8_t address, bool data)
		{
			writeBytes(address, {AddressType::BOOLEAN, 0, static_cast<uint8_t>(data)});
		}

		// Read
		template <>
		int32_t Conbus::read(uint8_t address)
		{
			return static_cast<int32_t>((this->_registers[_device][address][2] << 24) | (this->_registers[_device][address][3] << 16) | (this->_registers[_device][address][24] << 8) | this->_registers[_device][address][5]);
		}

		template <>
		float Conbus::read(uint8_t address)
		{
			auto data = static_cast<uint32_t>((this->_registers[_device][address][2] << 24) | (this->_registers[_device][address][3] << 16) | (this->_registers[_device][address][4] << 8) | this->_registers[_device][address][5]);
			float realData = data / FLOAT_PRECISION;
			return realData;
		}

		template <>
		bool Conbus::read(uint8_t address)
		{
			return this->_registers[_device][address][2] != 0;
		}

		// Write To
		void Conbus::publishWrite(Device device, uint8_t address, std::vector<uint8_t> data)
		{
			auto msg = autonav_msgs::msg::ConBusInstruction();
			msg.device = device;
			msg.address = address;
			msg.data = data;
			msg.opcode = ConbusOpcode::WRITE;
			_conbusPublisher->publish(msg);
		}

		void Conbus::writeTo(Device device, uint8_t address, int32_t data)
		{
			auto dataBytes = std::vector<uint8_t>();
			this->publishWrite(device, address, {AddressType::INTEGER, 0, static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, float data)
		{
			auto dataInt = static_cast<int32_t>(data * FLOAT_PRECISION);
			this->publishWrite(device, address, {AddressType::FLOAT, 0, static_cast<uint8_t>(dataInt >> 24), static_cast<uint8_t>(dataInt >> 16), static_cast<uint8_t>(dataInt >> 8), static_cast<uint8_t>(dataInt & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, bool data)
		{
			this->publishWrite(device, address, {AddressType::BOOLEAN, 0, static_cast<uint8_t>(data)});
		}

		// Read From

		void Conbus::publishRead(Device device, uint8_t address)
		{
			auto msg = autonav_msgs::msg::ConBusInstruction();
			msg.device = device;
			msg.address = address;
			msg.opcode = ConbusOpcode::READ;
			_conbusPublisher->publish(msg);
		}

		void Conbus::requestRemoteRegister(Device device, uint8_t address)
		{
			this->publishRead(device, address);
		}

		void Conbus::requestAllRemoteRegistersFrom(Device device)
		{
			auto msg = autonav_msgs::msg::ConBusInstruction();
			msg.device = device;
			msg.address = 0;
			msg.opcode = ConbusOpcode::READ_ALL;
			_conbusPublisher->publish(msg);
		}

		void Conbus::requestAllRemoteRegisters()
		{
			requestAllRemoteRegistersFrom(Autonav::Device::LOGGING);
			requestAllRemoteRegistersFrom(Autonav::Device::MANUAL_CONTROL);
			requestAllRemoteRegistersFrom(Autonav::Device::SERIAL_CAN);
			requestAllRemoteRegistersFrom(Autonav::Device::SERIAL_IMU);
			requestAllRemoteRegistersFrom(Autonav::Device::STEAM_TRANSLATOR);
		}

		template <>
		bool Conbus::read(Device device, uint8_t address)
		{
			return this->_registers[device][address][2] != 0;
		}

		template <>
		int32_t Conbus::read(Device device, uint8_t address)
		{
			return static_cast<int32_t>((this->_registers[device][address][2] << 24) | (this->_registers[device][address][3] << 16) | (this->_registers[device][address][4] << 8) | this->_registers[device][address][5]);
		}

		template <>
		float Conbus::read(Device device, uint8_t address)
		{
			auto data = (this->_registers[device][address][2] << 24) | (this->_registers[device][address][3] << 16) | (this->_registers[device][address][4] << 8) | this->_registers[device][address][5];
			float realData = data / FLOAT_PRECISION;
			return realData;
		}

		// Callbacks

		void Conbus::onConbusInstruction(const autonav_msgs::msg::ConBusInstruction::SharedPtr msg)
		{
			if (msg->opcode == ConbusOpcode::READ && msg->device == _device)
			{
				if (this->_registers.find(_device) == this->_registers.end())
				{
					return;
				}

				auto deviceRegister = this->_registers[_device];
				if (deviceRegister.find(msg->address) == deviceRegister.end())
				{
					return;
				}

				auto registerData = deviceRegister[msg->address];
				if (registerData.size() == 0)
				{
					return;
				}

				auto reply = autonav_msgs::msg::ConBusInstruction();
				reply.device = _device;
				reply.address = msg->address;
				reply.opcode = ConbusOpcode::READ_ACK;
				reply.data = registerData;
				_conbusPublisher->publish(reply);
			}

			if (msg->opcode == ConbusOpcode::READ_ACK)
			{
				if(this->_registers.find(msg->device) == this->_registers.end())
				{
					auto newRegister = std::map<uint8_t, std::vector<uint8_t>>();
					newRegister[msg->address] = msg->data;
					this->_registers[msg->device] = newRegister;
					return;
				}

				auto deviceRegister = this->_registers[msg->device];
				deviceRegister[msg->address] = msg->data;
				this->_registers[msg->device] = deviceRegister;
			}

			if (msg->opcode == ConbusOpcode::WRITE && msg->device == _device)
			{
				if(this->_registers.find(msg->device) == this->_registers.end())
				{
					auto deviceRegister = std::map<uint8_t, std::vector<uint8_t>>();
					deviceRegister[msg->address] = msg->data;
					this->_registers[msg->device] = deviceRegister;
					return;
				}

				auto deviceRegister = this->_registers[msg->device];
				deviceRegister[msg->address] = msg->data;
				this->_registers[msg->device] = deviceRegister;
				return;
			}

			if (msg->opcode == ConbusOpcode::WRITE_ACK)
			{
				if(this->_registers.find(msg->device) == this->_registers.end())
				{
					auto newRegister = std::map<uint8_t, std::vector<uint8_t>>();
					newRegister[msg->address] = msg->data;
					this->_registers[msg->device] = newRegister;
					return;
				}

				auto deviceRegister = this->_registers[msg->device];
				deviceRegister[msg->address] = msg->data;
				this->_registers[msg->device] = deviceRegister;
				return;
			}

			if (msg->opcode == ConbusOpcode::READ_ALL && msg->device == _device)
			{
				for (auto const &[address, data] : this->_registers[_device])
				{
					auto msg = autonav_msgs::msg::ConBusInstruction();
					msg.device = _device;
					msg.address = address;
					msg.opcode = ConbusOpcode::READ_ACK;
					msg.data = data;
					_conbusPublisher->publish(msg);
				}
			}
		}

		// Iterators
		
		std::map<uint8_t, std::map<uint8_t, std::vector<uint8_t>>>::iterator Conbus::begin()
		{
			return _registers.begin();
		}

		std::map<uint8_t, std::map<uint8_t, std::vector<uint8_t>>>::iterator Conbus::end()
		{
			return _registers.end();
		}
	}
}