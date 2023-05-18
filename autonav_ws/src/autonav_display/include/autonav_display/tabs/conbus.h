#include "autonav_msgs/msg/conbus.hpp"
#include "scr_core/configuration.h"
#include "scr_core/utils.h"
#include "scr_core/node.h"
#include "imgui.h"

namespace MotorController
{
	const uint8_t DeviceID = 0x10;

	const uint8_t UPDATE_PERIOD = 0x0;
	const uint8_t PULSES_PER_RADIAN = 0x01;
	const uint8_t WHEEL_RADIUS = 0x02;
	const uint8_t WHEEL_BASE_LENGTH = 0x03;
	const uint8_t SLEW_RATE_LIMIT = 0x04;
	
	const uint8_t VELOCITY_KP = 0x10;
	const uint8_t VELOCITY_KI = 0x11;
	const uint8_t VELOCITY_KD = 0x12;
	const uint8_t VELOCITY_KF = 0x13;

	const uint8_t ANGULAR_KP = 0x20;
	const uint8_t ANGULAR_KI = 0x21;
	const uint8_t ANGULAR_KD = 0x22;
	const uint8_t ANGULAR_KF = 0x23;

	const uint8_t USE_OBSTACLE_AVOIDANCE = 0x30;
	const uint8_t COLLISION_BOX_DISTANCE = 0x31;

	const uint8_t SEND_STATISTICS = 0x40;
}

struct ReadResponse {
	uint8_t device;
	uint8_t address;
	uint8_t length;
	uint8_t reserved;
	uint8_t* data;
};

struct WriteResponse {
	uint8_t device;
	uint8_t address;
	uint8_t length;
	uint8_t reserved;
	uint8_t* data;
};

autonav_msgs::msg::Conbus createReadInstruction(uint8_t device, uint8_t address)
{
	auto instruction = autonav_msgs::msg::Conbus();
	instruction.id = 1000 + device;
	instruction.data = { address };
	return instruction;
}

autonav_msgs::msg::Conbus createWriteInstruction(uint8_t device, uint8_t address, uint8_t length, uint8_t* data)
{
	auto instruction = autonav_msgs::msg::Conbus();
	instruction.id = 1200 + device;
	instruction.data = { address, length, 0 };
	for (int i = 0; i < length; i++)
	{
		instruction.data.push_back(data[i]);
	}
	return instruction;
}

ReadResponse createReadResponse(autonav_msgs::msg::Conbus::SharedPtr msg)
{
	auto response = ReadResponse();
	response.device = msg->id - 1000;
	response.address = msg->data[0];
	response.length = msg->data[1];
	response.reserved = msg->data[2];
	response.data = new uint8_t[response.length];
	for (int i = 0; i < response.length; i++)
	{
		response.data[i] = msg->data[3 + i];
	}
	return response;
}

WriteResponse createWriteResponse(autonav_msgs::msg::Conbus::SharedPtr msg)
{
	auto response = WriteResponse();
	response.device = msg->id - 1300;
	response.address = msg->data[0];
	response.length = msg->data[1];
	response.reserved = msg->data[2];
	response.data = new uint8_t[response.length];
	for (int i = 0; i < response.length; i++)
	{
		response.data[i] = msg->data[3 + i];
	}
	return response;
}

void ShowConbus(SCR::Node *node)
{
	static rclcpp::Publisher<autonav_msgs::msg::Conbus>::SharedPtr conbusPublisher = nullptr;
	static rclcpp::Subscription<autonav_msgs::msg::Conbus>::SharedPtr conbusSubscriber = nullptr;
	static std::vector<uint32_t> pendingRequests = {};
	static std::vector<uint32_t> completedRequests = {};
	static int totalResponses = 0;
	static std::map<uint8_t, std::map<uint8_t, uint8_t*>> values = {};

	if (conbusPublisher == nullptr)
	{
		conbusPublisher = node->create_publisher<autonav_msgs::msg::Conbus>("/autonav/conbus", 10);
		conbusSubscriber = node->create_subscription<autonav_msgs::msg::Conbus>("/autonav/conbus", 10, [](const autonav_msgs::msg::Conbus::SharedPtr msg) {
			totalResponses++;

			if (msg->id >= 1100 && msg->id < 1200)
			{
				auto response = createReadResponse(msg);
				values[response.device][response.address] = response.data;

				auto uniqueId = response.device % response.address * 1000 + 100;
				if (std::find(pendingRequests.begin(), pendingRequests.end(), uniqueId) != pendingRequests.end())
				{
					pendingRequests.erase(std::remove(pendingRequests.begin(), pendingRequests.end(), uniqueId), pendingRequests.end());
					completedRequests.push_back(uniqueId);
				}
				return;
			}

			if (msg->id >= 1300 && msg->id < 1400)
			{
				auto response = createWriteResponse(msg);
				values[response.device][response.address] = response.data;

				auto uniqueId = response.device % response.address * 2000 + 50;
				if (std::find(pendingRequests.begin(), pendingRequests.end(), uniqueId) != pendingRequests.end())
				{
					pendingRequests.erase(std::remove(pendingRequests.begin(), pendingRequests.end(), uniqueId), pendingRequests.end());
					completedRequests.push_back(uniqueId);
				}
				return;
			}
		});

		auto motorcontroller_readall = createReadInstruction(MotorController::DeviceID, 0xFF);
		pendingRequests.push_back(MotorController::DeviceID % 0xFF * 1000 + 100);
		conbusPublisher->publish(motorcontroller_readall);
	}

	if (ImGui::Button("Request Values"))
	{
		auto motorcontroller_readall = createReadInstruction(MotorController::DeviceID, 0xFF);
		pendingRequests.push_back(MotorController::DeviceID % 0xFF * 1000 + 100);
		conbusPublisher->publish(motorcontroller_readall);
	}

	ImGui::Text("Pending Requests: %d", pendingRequests.size());
	ImGui::Text("Completed Requests: %d", completedRequests.size());
	ImGui::Text("Total Responses: %d", totalResponses);

	if (values.find(MotorController::DeviceID) != values.end())
	{
		ImGui::SeparatorText("Motor Controller");
		auto motorcontroller_values = values[MotorController::DeviceID];

		// General
		if (motorcontroller_values.find(MotorController::UPDATE_PERIOD) != motorcontroller_values.end())
		{
			ImGui::Text("Update Period: %f", *(float*)motorcontroller_values[MotorController::UPDATE_PERIOD]);
		}

		if (motorcontroller_values.find(MotorController::PULSES_PER_RADIAN) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Pulses Per Radian", (float*)motorcontroller_values[MotorController::PULSES_PER_RADIAN]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::PULSES_PER_RADIAN, 4, motorcontroller_values[MotorController::PULSES_PER_RADIAN]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::PULSES_PER_RADIAN * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		if (motorcontroller_values.find(MotorController::WHEEL_RADIUS) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Wheel Radius", (float*)motorcontroller_values[MotorController::WHEEL_RADIUS]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::WHEEL_RADIUS, 4, motorcontroller_values[MotorController::WHEEL_RADIUS]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::WHEEL_RADIUS * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		if (motorcontroller_values.find(MotorController::WHEEL_BASE_LENGTH) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Wheel Base Length", (float*)motorcontroller_values[MotorController::WHEEL_BASE_LENGTH]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::WHEEL_BASE_LENGTH, 4, motorcontroller_values[MotorController::WHEEL_BASE_LENGTH]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::WHEEL_BASE_LENGTH * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		if (motorcontroller_values.find(MotorController::SLEW_RATE_LIMIT) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Slew Rate Limit", (float*)motorcontroller_values[MotorController::SLEW_RATE_LIMIT]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::SLEW_RATE_LIMIT, 4, motorcontroller_values[MotorController::SLEW_RATE_LIMIT]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::SLEW_RATE_LIMIT * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		// Velocity PID
		if (motorcontroller_values.find(MotorController::VELOCITY_KI) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Velocity KI", (float*)motorcontroller_values[MotorController::VELOCITY_KI]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::VELOCITY_KI, 4, motorcontroller_values[MotorController::VELOCITY_KI]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::VELOCITY_KI * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		if (motorcontroller_values.find(MotorController::VELOCITY_KP) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Velocity KP", (float*)motorcontroller_values[MotorController::VELOCITY_KP]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::VELOCITY_KP, 4, motorcontroller_values[MotorController::VELOCITY_KP]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::VELOCITY_KP * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		if (motorcontroller_values.find(MotorController::VELOCITY_KD) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Velocity KD", (float*)motorcontroller_values[MotorController::VELOCITY_KD]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::VELOCITY_KD, 4, motorcontroller_values[MotorController::VELOCITY_KD]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::VELOCITY_KD * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		if (motorcontroller_values.find(MotorController::VELOCITY_KF) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Velocity KF", (float*)motorcontroller_values[MotorController::VELOCITY_KF]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::VELOCITY_KF, 4, motorcontroller_values[MotorController::VELOCITY_KF]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::VELOCITY_KF * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		// Angular PID
		if (motorcontroller_values.find(MotorController::ANGULAR_KI) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Angular KI", (float*)motorcontroller_values[MotorController::ANGULAR_KI]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::ANGULAR_KI, 4, motorcontroller_values[MotorController::ANGULAR_KI]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::ANGULAR_KI * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		if (motorcontroller_values.find(MotorController::ANGULAR_KP) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Angular KP", (float*)motorcontroller_values[MotorController::ANGULAR_KP]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::ANGULAR_KP, 4, motorcontroller_values[MotorController::ANGULAR_KP]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::ANGULAR_KP * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		if (motorcontroller_values.find(MotorController::ANGULAR_KD) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Angular KD", (float*)motorcontroller_values[MotorController::ANGULAR_KD]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::ANGULAR_KD, 4, motorcontroller_values[MotorController::ANGULAR_KD]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::ANGULAR_KD * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		if (motorcontroller_values.find(MotorController::ANGULAR_KF) != motorcontroller_values.end())
		{
			if (ImGui::InputFloat("Angular KF", (float*)motorcontroller_values[MotorController::ANGULAR_KF]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::ANGULAR_KF, 4, motorcontroller_values[MotorController::ANGULAR_KF]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::ANGULAR_KF * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		// Obstacle Avoidance
		if (motorcontroller_values.find(MotorController::USE_OBSTACLE_AVOIDANCE) != motorcontroller_values.end())
		{
			if (ImGui::Checkbox("Use Obstacle Avoidance", (bool*)motorcontroller_values[MotorController::USE_OBSTACLE_AVOIDANCE]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::USE_OBSTACLE_AVOIDANCE, 1, motorcontroller_values[MotorController::USE_OBSTACLE_AVOIDANCE]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::USE_OBSTACLE_AVOIDANCE * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		if (motorcontroller_values.find(MotorController::COLLISION_BOX_DISTANCE) != motorcontroller_values.end())
		{
			if (ImGui::InputInt("Obstacle Avoidance Distance", (int*)motorcontroller_values[MotorController::COLLISION_BOX_DISTANCE]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::COLLISION_BOX_DISTANCE, 4, motorcontroller_values[MotorController::COLLISION_BOX_DISTANCE]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::COLLISION_BOX_DISTANCE * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}

		// Send Statistics
		if (motorcontroller_values.find(MotorController::SEND_STATISTICS) != motorcontroller_values.end())
		{
			if (ImGui::Checkbox("Use Obstacle Avoidance", (bool*)motorcontroller_values[MotorController::SEND_STATISTICS]))
			{
				auto motorcontroller_write = createWriteInstruction(MotorController::DeviceID, MotorController::SEND_STATISTICS, 1, motorcontroller_values[MotorController::SEND_STATISTICS]);
				pendingRequests.push_back(MotorController::DeviceID % MotorController::SEND_STATISTICS * 2000 + 50);
				conbusPublisher->publish(motorcontroller_write);
			}
		}
	}
}