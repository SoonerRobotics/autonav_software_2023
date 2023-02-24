#include "autonav_libs/common.h"

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

			// _conbusSubscriber = node.create_subscription<autonav_msgs::msg::ConBusInstruction>("/autonav/conbus", 10, std::bind(&Conbus::onConbus, this, std::placeholders::_1));
			_conbusPublisher = node->create_publisher<autonav_msgs::msg::ConBusInstruction>("/autonav/conbus", 10);
		}

		Conbus::~Conbus()
		{
		}

		// TODO: Tell everyone else about our config changes!
		void Conbus::write(uint8_t address, std::vector<uint8_t> data)
		{
			this->_registers[_device][address] = data;
		}

		void Conbus::write(uint8_t address, uint8_t data)
		{
			this->_registers[_device][address] = {data};
		}

		void Conbus::write(uint8_t address, uint16_t data)
		{
			this->_registers[_device][address] = {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)};
		}

		void Conbus::write(uint8_t address, uint32_t data)
		{
			this->_registers[_device][address] = {static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)};
		}

		void Conbus::write(uint8_t address, uint64_t data)
		{
			this->_registers[_device][address] = {static_cast<uint8_t>(data >> 56), static_cast<uint8_t>(data >> 48), static_cast<uint8_t>(data >> 40), static_cast<uint8_t>(data >> 32), static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)};
		}

		void Conbus::write(uint8_t address, int8_t data)
		{
			this->_registers[_device][address] = {static_cast<uint8_t>(data)};
		}

		void Conbus::write(uint8_t address, int16_t data)
		{
			this->_registers[_device][address] = {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)};
		}

		void Conbus::write(uint8_t address, int32_t data)
		{
			this->_registers[_device][address] = {static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)};
		}

		void Conbus::write(uint8_t address, int64_t data)
		{
			this->_registers[_device][address] = {static_cast<uint8_t>(data >> 56), static_cast<uint8_t>(data >> 48), static_cast<uint8_t>(data >> 40), static_cast<uint8_t>(data >> 32), static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)};
		}

		void Conbus::write(uint8_t address, float data)
		{
			auto dataInt = *reinterpret_cast<uint32_t *>(&data) & 0xFFFFFFFF;
			this->_registers[_device][address] = {static_cast<uint8_t>(dataInt >> 24), static_cast<uint8_t>(dataInt >> 16), static_cast<uint8_t>(dataInt >> 8), static_cast<uint8_t>(dataInt & 0xFF)};
		}

		void Conbus::write(uint8_t address, bool data)
		{
			this->_registers[_device][address] = {static_cast<uint8_t>(data)};
		}

		// Read

		template <>
		std::vector<uint8_t> Conbus::read(uint8_t address)
		{
			return this->_registers[_device][address];
		}

		template <>
		uint8_t Conbus::read(uint8_t address)
		{
			return this->_registers[_device][address][0];
		}

		template <>
		uint16_t Conbus::read(uint8_t address)
		{
			return (this->_registers[_device][address][0] << 8) | this->_registers[_device][address][1];
		}

		template <>
		uint32_t Conbus::read(uint8_t address)
		{
			return (this->_registers[_device][address][0] << 24) | (this->_registers[_device][address][1] << 16) | (this->_registers[_device][address][2] << 8) | this->_registers[_device][address][3];
		}

		template <>
		uint64_t Conbus::read(uint8_t address)
		{
			return (this->_registers[_device][address][0] << 56) | (this->_registers[_device][address][1] << 48) | (this->_registers[_device][address][2] << 40) | (this->_registers[_device][address][3] << 32) | (this->_registers[_device][address][4] << 24) | (this->_registers[_device][address][5] << 16) | (this->_registers[_device][address][6] << 8) | this->_registers[_device][address][7];
		}

		template <>
		int8_t Conbus::read(uint8_t address)
		{
			return static_cast<int8_t>(this->_registers[_device][address][0]);
		}

		template <>
		int16_t Conbus::read(uint8_t address)
		{
			return static_cast<int16_t>((this->_registers[_device][address][0] << 8) | this->_registers[_device][address][1]);
		}

		template <>
		int32_t Conbus::read(uint8_t address)
		{
			return static_cast<int32_t>((this->_registers[_device][address][0] << 24) | (this->_registers[_device][address][1] << 16) | (this->_registers[_device][address][2] << 8) | this->_registers[_device][address][3]);
		}

		template <>
		int64_t Conbus::read(uint8_t address)
		{
			return static_cast<int64_t>((this->_registers[_device][address][0] << 56) | (this->_registers[_device][address][1] << 48) | (this->_registers[_device][address][2] << 40) | (this->_registers[_device][address][3] << 32) | (this->_registers[_device][address][4] << 24) | (this->_registers[_device][address][5] << 16) | (this->_registers[_device][address][6] << 8) | this->_registers[_device][address][7]);
		}

		template <>
		float Conbus::read(uint8_t address)
		{
			auto dataInt = (this->_registers[_device][address][0] << 24) | (this->_registers[_device][address][1] << 16) | (this->_registers[_device][address][2] << 8) | this->_registers[_device][address][3];
			dataInt &= 0xFFFFFFFF;
			return *reinterpret_cast<float *>(&dataInt);
		}

		template <>
		bool Conbus::read(uint8_t address)
		{
			return this->_registers[_device][address][0] != 0;
		}

		// Write To

		void Conbus::publishWrite(Device device, uint8_t address, std::vector<uint8_t> data)
		{
			auto msg = autonav_msgs::msg::ConBusInstruction();
			msg.device = device;
			msg.address = address;
			msg.data = data;
			msg.opcode = 2;
			_conbusPublisher->publish(msg);
		}

		void Conbus::writeTo(Device device, uint8_t address, std::vector<uint8_t> data)
		{
			this->publishWrite(device, address, data);
		}

		void Conbus::writeTo(Device device, uint8_t address, uint8_t data)
		{
			this->publishWrite(device, address, {data});
		}

		void Conbus::writeTo(Device device, uint8_t address, uint16_t data)
		{
			this->publishWrite(device, address, {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, uint32_t data)
		{
			this->publishWrite(device, address, {static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, uint64_t data)
		{
			this->publishWrite(device, address, {static_cast<uint8_t>(data >> 56), static_cast<uint8_t>(data >> 48), static_cast<uint8_t>(data >> 40), static_cast<uint8_t>(data >> 32), static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, int8_t data)
		{
			this->publishWrite(device, address, {static_cast<uint8_t>(data)});
		}

		void Conbus::writeTo(Device device, uint8_t address, int16_t data)
		{
			this->publishWrite(device, address, {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, int32_t data)
		{
			this->publishWrite(device, address, {static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, int64_t data)
		{
			this->publishWrite(device, address, {static_cast<uint8_t>(data >> 56), static_cast<uint8_t>(data >> 48), static_cast<uint8_t>(data >> 40), static_cast<uint8_t>(data >> 32), static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, float data)
		{
			auto dataInt = *reinterpret_cast<uint32_t *>(&data);
			dataInt &= 0xFFFFFFFF;
			this->publishWrite(device, address, {static_cast<uint8_t>(dataInt >> 24), static_cast<uint8_t>(dataInt >> 16), static_cast<uint8_t>(dataInt >> 8), static_cast<uint8_t>(dataInt & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, bool data)
		{
			this->publishWrite(device, address, {static_cast<uint8_t>(data)});
		}

		// Read From

		void Conbus::publishRead(Device device, uint8_t address)
		{
			auto msg = autonav_msgs::msg::ConBusInstruction();
			msg.device = device;
			msg.address = address;
			msg.opcode = 0;
			_conbusPublisher->publish(msg);
		}

		void Conbus::requestRemoteRegister(Device device, uint8_t address)
		{
			this->publishRead(device, address);
		}

		void Conbus::requestAllRemoteRegisters(Device device)
		{
			auto msg = autonav_msgs::msg::ConBusInstruction();
			msg.device = device;
			msg.address = 0;
			msg.opcode = 5;
			_conbusPublisher->publish(msg);
		}

		// Callbacks

		void Conbus::onConbusInstruction(const autonav_msgs::msg::ConBusInstruction::SharedPtr msg)
		{
			if (msg->opcode == 0 && msg->device == _device)
			{
				auto registerData = this->_registers[_device][msg->address];
				auto reply = autonav_msgs::msg::ConBusInstruction();
				reply.device = _device;
				reply.address = msg->address;
				reply.opcode = 1;
				reply.data = registerData;
				_conbusPublisher->publish(reply);
				return;
			}

			if (msg->opcode == 1)
			{
				auto deviceRegisters = this->_registers[msg->device];
				deviceRegisters[msg->address] = msg->data;
				this->_registers[msg->device] = deviceRegisters;
				return;
			}

			if (msg->opcode == 2)
			{
				// Write to a register
				auto deviceRegisters = this->_registers[msg->device];
				deviceRegisters[msg->address] = msg->data;
				this->_registers[msg->device] = deviceRegisters;
				return;
			}

			if (msg->opcode == 5 && msg->device == _device)
			{
				// Read all of our registers and publish them as opcode 1
				for (auto const &[address, data] : this->_registers[_device])
				{
					auto msg = autonav_msgs::msg::ConBusInstruction();
					msg.device = _device;
					msg.address = address;
					msg.opcode = 1;
					msg.data = data;
					_conbusPublisher->publish(msg);
				}
			}
		}
	}
}