#include "autonav_libs/common.h"
#include <unistd.h>

namespace Autonav
{
	namespace ROS
	{
		AutoNode::AutoNode(Autonav::Device device, std::string node_name)
			: Node(node_name), _config(device, this)
		{
			_device = device;
			_systemState = State::SystemState::DISABLED;
			_deviceState = State::DeviceState::ALIVE;

			_systemStateSubscriber = this->create_subscription<autonav_msgs::msg::SystemState>("/autonav/state/system", 10, std::bind(&AutoNode::onSystemState, this, std::placeholders::_1));
			_deviceStateSubscriber = this->create_subscription<autonav_msgs::msg::DeviceState>("/autonav/state/device", 10, std::bind(&AutoNode::onDeviceState, this, std::placeholders::_1));

			_deviceStateClient = this->create_client<autonav_msgs::srv::SetDeviceState>("/autonav/state/set_device_state");
			_systemStateClient = this->create_client<autonav_msgs::srv::SetSystemState>("/autonav/state/set_system_state");

			_aliveTimer = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&AutoNode::onAliveTimer, this));
		}

		AutoNode::~AutoNode()
		{
		}

		void AutoNode::onAliveTimer()
		{
			if (_deviceState != State::DeviceState::ALIVE)
			{
				_aliveTimer->cancel();
				return;
			}

			this->setDeviceState(State::DeviceState::ALIVE);
		}

		bool AutoNode::setSystemState(State::SystemState state)
		{
			auto request = std::make_shared<autonav_msgs::srv::SetSystemState::Request>();
			request->state = static_cast<uint8_t>(state);

			while (!_systemStateClient->wait_for_service(std::chrono::seconds(1)))
			{
				if (!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
					return false;
				}
				
				RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
			}
			
			auto result = _systemStateClient->async_send_request(request);
			return false;
		}

		bool AutoNode::setDeviceState(State::DeviceState state)
		{
			auto request = std::make_shared<autonav_msgs::srv::SetDeviceState::Request>();
			request->device = static_cast<uint8_t>(_device);
			request->state = static_cast<uint8_t>(state);

			while (!_deviceStateClient->wait_for_service(std::chrono::seconds(1)))
			{
				if (!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
					return false;
				}
				
				RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
			}
			
			auto result = _deviceStateClient->async_send_request(request);
			return false;	
		}

		void AutoNode::onSystemState(const autonav_msgs::msg::SystemState::SharedPtr msg)
		{
			_systemState = static_cast<State::SystemState>(msg->state);

			if(_systemState != State::SystemState::DISABLED && _deviceState == State::DeviceState::READY)
			{
				this->setDeviceState(State::DeviceState::OPERATING);
			}
		}

		void AutoNode::onDeviceState(const autonav_msgs::msg::DeviceState::SharedPtr msg)
		{
			if (static_cast<Autonav::Device>(msg->device) != _device)
			{
				return;
			}

			auto newState = static_cast<State::DeviceState>(msg->state);
			if (newState == State::DeviceState::STANDBY && _deviceState == State::DeviceState::ALIVE)
			{
				_deviceState = newState;
				this->setup();
				return;
			}

			if (newState == State::DeviceState::OPERATING && _deviceState != State::DeviceState::OPERATING)
			{
				_deviceState = newState;
				this->operate();
				return;
			}

			if(newState == State::DeviceState::ALIVE)
			{
				_aliveTimer->reset();
				_deviceState = newState;
				return;
			}

			_deviceState = newState;
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
			writeBytes(address, {0, static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::write(uint8_t address, float data)
		{
			auto dataInt = static_cast<int32_t>(data * FLOAT_PRECISION);
			writeBytes(address, {0, static_cast<uint8_t>(dataInt >> 24), static_cast<uint8_t>(dataInt >> 16), static_cast<uint8_t>(dataInt >> 8), static_cast<uint8_t>(dataInt & 0xFF)});
		}

		void Conbus::write(uint8_t address, bool data)
		{
			writeBytes(address, {0, static_cast<uint8_t>(data)});
		}

		// Read
		template <>
		int32_t Conbus::read(uint8_t address)
		{
			return static_cast<int32_t>((this->_registers[_device][address][1] << 24) | (this->_registers[_device][address][2] << 16) | (this->_registers[_device][address][3] << 8) | this->_registers[_device][address][4]);
		}

		template <>
		float Conbus::read(uint8_t address)
		{
			auto data = static_cast<int32_t>((this->_registers[_device][address][1] << 24) | (this->_registers[_device][address][2] << 16) | (this->_registers[_device][address][3] << 8) | this->_registers[_device][address][4]);
			float realData = data / FLOAT_PRECISION;
			return realData;
		}

		template <>
		bool Conbus::read(uint8_t address)
		{
			return this->_registers[_device][address][1] != 0;
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
			this->publishWrite(device, address, {0, static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, float data)
		{
			auto dataInt = static_cast<int32_t>(data * FLOAT_PRECISION);
			this->publishWrite(device, address, {0, static_cast<uint8_t>(dataInt >> 24), static_cast<uint8_t>(dataInt >> 16), static_cast<uint8_t>(dataInt >> 8), static_cast<uint8_t>(dataInt & 0xFF)});
		}

		void Conbus::writeTo(Device device, uint8_t address, bool data)
		{
			this->publishWrite(device, address, {0, static_cast<uint8_t>(data)});
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
			requestAllRemoteRegistersFrom(Autonav::Device::DISPLAY_NODE);
			requestAllRemoteRegistersFrom(Autonav::Device::LOGGING);
			requestAllRemoteRegistersFrom(Autonav::Device::MANUAL_CONTROL_STEAM);
			requestAllRemoteRegistersFrom(Autonav::Device::MANUAL_CONTROL_XBOX);
			requestAllRemoteRegistersFrom(Autonav::Device::SERIAL_CAN);
			requestAllRemoteRegistersFrom(Autonav::Device::SERIAL_IMU);
			requestAllRemoteRegistersFrom(Autonav::Device::STEAM_TRANSLATOR);
			requestAllRemoteRegistersFrom(Autonav::Device::CAMERA_TRANSLATOR);
		}

		template <>
		bool Conbus::read(Device device, uint8_t address)
		{
			return this->_registers[device][address][1] != 0;
		}

		template <>
		int32_t Conbus::read(Device device, uint8_t address)
		{
			return static_cast<int32_t>((this->_registers[device][address][1] << 24) | (this->_registers[device][address][2] << 16) | (this->_registers[device][address][3] << 8) | this->_registers[device][address][4]);
		}

		template <>
		float Conbus::read(Device device, uint8_t address)
		{
			auto data = (this->_registers[device][address][1] << 24) | (this->_registers[device][address][2] << 16) | (this->_registers[device][address][3] << 8) | this->_registers[device][address][4];
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
				if (this->_registers.find(msg->device) == this->_registers.end())
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
				if (this->_registers.find(msg->device) == this->_registers.end())
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
				if (this->_registers.find(msg->device) == this->_registers.end())
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

		std::map<uint8_t, std::vector<uint8_t>> Conbus::getRegistersForDevice(Device device)
		{
			if (this->_registers.find(device) == this->_registers.end())
			{
				return std::map<uint8_t, std::vector<uint8_t>>();
			}

			return _registers[device];
		}
	}
}