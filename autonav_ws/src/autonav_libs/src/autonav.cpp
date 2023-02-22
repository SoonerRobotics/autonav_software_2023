#include "autonav_libs/autonav.h"

using namespace Autonav::ConBus;

Controller::Controller()
	: _publisher(nullptr), _device(-1)
{
	_registers.resize(100);
}

Controller::Controller(int device, rclcpp::Publisher<autonav_msgs::msg::ConBusInstruction>::SharedPtr publisher)
	: _publisher(publisher), _device(device)
{
	_registers.resize(100);
}

Controller::~Controller()
{
}

void Controller::read(uint8_t device, uint8_t address, register_entry *entry)
{
	*entry = _registers[address];
}

void Controller::write(uint8_t device, uint8_t address, uint8_t *data, uint8_t length)
{
	_registers[address].address = address;
	_registers[address].length = length;
	_registers[address].data = new uint8_t[length];
	for (int i = 0; i < length; i++)
	{
		_registers[address].data[i] = data[i];
	}
}

// Create definition for Controller::onInstruction
void Controller::onInstruction(const autonav_msgs::msg::ConBusInstruction::SharedPtr msg)
{
	if (msg->device != _device)
	{
		return;
	}

	switch (msg->opcode)
	{
	case Opcode::READ:
		register_entry entry;
		read(msg->device, msg->address, &entry);

		if (entry.data == nullptr || entry.length == 0)
		{
			return;
		}

		msg->opcode = Opcode::READ_ACK;
		_publisher->publish(*msg);
		break;

	case Opcode::WRITE:
		write(msg->device, msg->address, msg->data.data(), msg->data.size());
		msg->opcode = Opcode::WRITE_ACK;
		RCLCPP_INFO(rclcpp::get_logger("conbus"), "Write: %d %d %d | %d", msg->opcode, msg->device, msg->address, msg->data.size());
		_publisher->publish(*msg);
		break;

	case Opcode::READ_ALL:
		for (int i = 0; i < _registers.size(); i++)
		{
			if (_registers[i].data != nullptr && _registers[i].length > 0)
			{
				msg->opcode = Opcode::READ_ACK;
				msg->address = _registers[i].address;
				msg->data = std::vector<uint8_t>(_registers[i].data, _registers[i].data + _registers[i].length);
				_publisher->publish(*msg);
			}
		}
		break;
	}
}