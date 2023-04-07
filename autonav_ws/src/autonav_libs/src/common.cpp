#include "autonav_libs/common.h"
#include <unistd.h>

namespace Autonav
{
	namespace ROS
	{
		AutoNode::AutoNode(std::string node_name) : Node(node_name), config(node_name, this)
		{
			m_id = Autonav::hash(node_name);

			m_systemState = State::SystemState::DISABLED;
			m_deviceStates[m_id] = State::DeviceState::OFF;

			m_systemStateSubscriber = this->create_subscription<autonav_msgs::msg::SystemState>("/autonav/state/system", 10, std::bind(&AutoNode::onSystemState, this, std::placeholders::_1));
			m_deviceStateSubscriber = this->create_subscription<autonav_msgs::msg::DeviceState>("/autonav/state/device", 10, std::bind(&AutoNode::onDeviceState, this, std::placeholders::_1));

			m_deviceStateClient = this->create_client<autonav_msgs::srv::SetDeviceState>("/autonav/state/set_device_state");
			m_systemStateClient = this->create_client<autonav_msgs::srv::SetSystemState>("/autonav/state/set_system_state");

			_initializeTimer = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&AutoNode::onInitializeTimer, this));
		}

		AutoNode::~AutoNode()
		{
			m_systemStateSubscriber.reset();
			m_deviceStateSubscriber.reset();
			m_deviceStateClient.reset();
			m_systemStateClient.reset();
			_initializeTimer.reset();
		}

		void AutoNode::onInitializeTimer()
		{
			if (getDeviceState() != State::DeviceState::OFF)
			{
				_initializeTimer->cancel();
				return;
			}

			this->setDeviceState(State::DeviceState::STANDBY);
		}

		void AutoNode::terminate()
		{
			kill(getpid(), SIGKILL);
		}

		bool AutoNode::setSystemState(State::SystemState state)
		{
			auto request = std::make_shared<autonav_msgs::srv::SetSystemState::Request>();
			request->state = static_cast<uint8_t>(state);

			while (!m_systemStateClient->wait_for_service(std::chrono::seconds(1)))
			{
				if (!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
					return false;
				}

				RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
			}

			auto result = m_systemStateClient->async_send_request(request);
			return false;
		}

		bool AutoNode::setDeviceState(State::DeviceState state)
		{
			auto request = std::make_shared<autonav_msgs::srv::SetDeviceState::Request>();
			request->device = m_id;
			request->state = static_cast<uint8_t>(state);

			while (!m_deviceStateClient->wait_for_service(std::chrono::seconds(1)))
			{
				if (!rclcpp::ok())
				{
					RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
					return false;
				}

				RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
			}

			auto result = m_deviceStateClient->async_send_request(request);
			return false;
		}

		void AutoNode::onSystemState(const autonav_msgs::msg::SystemState::SharedPtr msg)
		{
			m_systemState = static_cast<State::SystemState>(msg->state);
			m_isSimulator = msg->is_simulator;
			
			if (m_systemState == State::SystemState::SHUTDOWN)
			{
				this->terminate();
			}
		}

		void AutoNode::onDeviceState(const autonav_msgs::msg::DeviceState::SharedPtr msg)
		{
			auto originalState = getDeviceState();
			m_deviceStates[msg->device] = static_cast<State::DeviceState>(msg->state);
			if (msg->device != m_id)
			{
				return;
			}

			auto newState = getDeviceState();
			if (newState == originalState)
			{
				return;
			}

			if (newState == State::DeviceState::STANDBY && originalState == State::DeviceState::OFF)
			{
				this->setup();
			}

			if (newState == State::DeviceState::OPERATING && originalState == State::DeviceState::READY)
			{
				this->operate();
			}

			if (newState == State::DeviceState::READY && originalState == State::DeviceState::OPERATING)
			{
				this->deoperate();
			}
		}

		State::SystemState AutoNode::getSystemState()
		{
			return m_systemState;
		}

		State::DeviceState AutoNode::getDeviceState()
		{
			return m_deviceStates[m_id];
		}

		State::DeviceState AutoNode::getDeviceState(std::string id)
		{
			return m_deviceStates[Autonav::hash(id)];
		}
	}

	namespace Configuration
	{
		Conbus::Conbus(std::string device, rclcpp::Node *node)
		{
			m_id = Autonav::hash(device);

			m_conbusSubscriber = node->create_subscription<autonav_msgs::msg::ConBusInstruction>("/autonav/conbus", 10, std::bind(&Conbus::onConbusInstruction, this, std::placeholders::_1));
			m_conbusPublisher = node->create_publisher<autonav_msgs::msg::ConBusInstruction>("/autonav/conbus", 10);
		}

		Conbus::~Conbus()
		{
		}

		void Conbus::writeBytes(uint8_t address, std::vector<uint8_t> data)
		{
			m_registers[m_id][address] = data;

			auto msg = autonav_msgs::msg::ConBusInstruction();
			msg.device = m_id;
			msg.address = address;
			msg.data = data;
			msg.opcode = ConbusOpcode::READ_ACK;
			m_conbusPublisher->publish(msg);
		}

		void Conbus::write(uint8_t address, int32_t data)
		{
			writeBytes(address, {0, static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::write(uint8_t address, float data)
		{
			unsigned char const *dataChar = reinterpret_cast<unsigned char const *>(&data);
			writeBytes(address, {0, dataChar[0], dataChar[1], dataChar[2], dataChar[3]});
		}

		void Conbus::write(uint8_t address, bool data)
		{
			writeBytes(address, {0, static_cast<uint8_t>(data)});
		}

		// Read
		template <>
		int32_t Conbus::read(uint8_t address)
		{
			return static_cast<int32_t>((this->m_registers[m_id][address][1] << 24) | (this->m_registers[m_id][address][2] << 16) | (this->m_registers[m_id][address][3] << 8) | this->m_registers[m_id][address][4]);
		}

		template <>
		float Conbus::read(uint8_t address)
		{
			unsigned char byte_array[] = {
				this->m_registers[m_id][address][1],
				this->m_registers[m_id][address][2],
				this->m_registers[m_id][address][3],
				this->m_registers[m_id][address][4]
			};
			float result;
			std::copy(
				reinterpret_cast<const char*>(&byte_array[0]),
				reinterpret_cast<const char*>(&byte_array[4]),
				reinterpret_cast<char*>(&result)
			);
			return result;
		}

		template <>
		std::vector<uint8_t> Conbus::read(uint8_t address)
		{
			return this->m_registers[m_id][address];
		}

		template <>
		bool Conbus::read(uint8_t address)
		{
			return this->m_registers[m_id][address][1] != 0;
		}

		// Write To
		void Conbus::publishWrite(std::string device, uint8_t address, std::vector<uint8_t> data)
		{
			auto msg = autonav_msgs::msg::ConBusInstruction();
			msg.device = Autonav::hash(device);
			msg.address = address;
			msg.data = data;
			msg.opcode = ConbusOpcode::WRITE;
			m_conbusPublisher->publish(msg);

			// Set it lcally
			m_registers[Autonav::hash(device)][address] = data;
		}

		void Conbus::writeTo(std::string device, uint8_t address, int32_t data)
		{
			auto dataBytes = std::vector<uint8_t>();
			this->publishWrite(device, address, {0, static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16), static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)});
		}

		void Conbus::writeTo(std::string device, uint8_t address, float data)
		{
			unsigned char const *dataChar = reinterpret_cast<unsigned char const *>(&data);
			this->publishWrite(device, address, {0, dataChar[0], dataChar[1], dataChar[2], dataChar[3]});
		}

		void Conbus::writeTo(std::string device, uint8_t address, bool data)
		{
			this->publishWrite(device, address, {0, static_cast<uint8_t>(data)});
		}

		void Conbus::writeTo(std::string device, uint8_t address, std::vector<uint8_t> data)
		{
			this->publishWrite(device, address, data);
		}

		// Read From

		void Conbus::publishRead(std::string device, uint8_t address)
		{
			auto msg = autonav_msgs::msg::ConBusInstruction();
			msg.device = Autonav::hash(device);
			msg.address = address;
			msg.opcode = ConbusOpcode::READ;
			m_conbusPublisher->publish(msg);
		}

		template <>
		bool Conbus::read(std::string device, uint8_t address)
		{
			return this->m_registers[Autonav::hash(device)][address][1] != 0;
		}

		template <>
		int32_t Conbus::read(std::string device, uint8_t address)
		{
			int64_t hash = Autonav::hash(device);
			return static_cast<int32_t>((this->m_registers[hash][address][1] << 24) | (this->m_registers[hash][address][2] << 16) | (this->m_registers[hash][address][3] << 8) | this->m_registers[hash][address][4]);
		}

		template <>
		float Conbus::read(std::string device, uint8_t address)
		{
			int64_t hash = Autonav::hash(device);
			unsigned char byte_array[] = {
				this->m_registers[hash][address][1],
				this->m_registers[hash][address][2],
				this->m_registers[hash][address][3],
				this->m_registers[hash][address][4]
			};
			float result;
			std::copy(
				reinterpret_cast<const char*>(&byte_array[0]),
				reinterpret_cast<const char*>(&byte_array[4]),
				reinterpret_cast<char*>(&result)
			);
			return result;
		}

		template <>
		std::vector<uint8_t> Conbus::read(std::string device, uint8_t address)
		{
			return this->m_registers[Autonav::hash(device)][address];
		}

		// Callbacks

		void Conbus::onConbusInstruction(const autonav_msgs::msg::ConBusInstruction::SharedPtr msg)
		{
			if (msg->opcode == ConbusOpcode::READ && msg->device == m_id)
			{
				if (this->m_registers.find(m_id) == this->m_registers.end())
				{
					return;
				}

				auto deviceRegister = this->m_registers[m_id];
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
				reply.device = m_id;
				reply.address = msg->address;
				reply.opcode = ConbusOpcode::READ_ACK;
				reply.data = registerData;
				m_conbusPublisher->publish(reply);
			}

			if (msg->opcode == ConbusOpcode::READ_ACK)
			{
				if (this->m_registers.find(msg->device) == this->m_registers.end())
				{
					auto newRegister = std::map<uint8_t, std::vector<uint8_t>>();
					newRegister[msg->address] = msg->data;
					this->m_registers[msg->device] = newRegister;
					return;
				}

				auto deviceRegister = this->m_registers[msg->device];
				deviceRegister[msg->address] = msg->data;
				this->m_registers[msg->device] = deviceRegister;
			}

			if (msg->opcode == ConbusOpcode::WRITE && msg->device == m_id)
			{
				if (this->m_registers.find(msg->device) == this->m_registers.end())
				{
					auto deviceRegister = std::map<uint8_t, std::vector<uint8_t>>();
					deviceRegister[msg->address] = msg->data;
					this->m_registers[msg->device] = deviceRegister;
					return;
				}

				auto deviceRegister = this->m_registers[msg->device];
				deviceRegister[msg->address] = msg->data;
				this->m_registers[msg->device] = deviceRegister;
				return;
			}

			if (msg->opcode == ConbusOpcode::WRITE_ACK)
			{
				if (this->m_registers.find(msg->device) == this->m_registers.end())
				{
					auto newRegister = std::map<uint8_t, std::vector<uint8_t>>();
					newRegister[msg->address] = msg->data;
					this->m_registers[msg->device] = newRegister;
					return;
				}

				auto deviceRegister = this->m_registers[msg->device];
				deviceRegister[msg->address] = msg->data;
				this->m_registers[msg->device] = deviceRegister;
				return;
			}

			if (msg->opcode == ConbusOpcode::READ_ALL && msg->device == m_id)
			{
				for (auto const &[address, data] : this->m_registers[m_id])
				{
					auto msg = autonav_msgs::msg::ConBusInstruction();
					msg.device = m_id;
					msg.address = address;
					msg.opcode = ConbusOpcode::READ_ACK;
					msg.data = data;
					m_conbusPublisher->publish(msg);
				}
			}
		}

		// Iterators

		std::map<uint8_t, std::map<uint8_t, std::vector<uint8_t>>>::iterator Conbus::begin()
		{
			return m_registers.begin();
		}

		std::map<uint8_t, std::map<uint8_t, std::vector<uint8_t>>>::iterator Conbus::end()
		{
			return m_registers.end();
		}

		std::map<uint8_t, std::vector<uint8_t>> Conbus::getRegistersForDevice(std::string device)
		{
			if (this->m_registers.find(Autonav::hash(device)) == this->m_registers.end())
			{
				return std::map<uint8_t, std::vector<uint8_t>>();
			}

			return m_registers[Autonav::hash(device)];
		}
	}
}