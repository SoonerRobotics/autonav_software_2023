#include "scr_core/configuration.h"
#include "scr_msgs/msg/log.hpp"
#include "scr_core/utils.h"
#include "scr_core/node.h"
#include "imgui.h"

void ShowLogs(SCR::Node *node)
{
	static rclcpp::Subscription<scr_msgs::msg::Log>::SharedPtr logSubscriber = nullptr;
	static std::vector<scr_msgs::msg::Log> logs;

	if (logSubscriber == nullptr)
	{
		logSubscriber = node->create_subscription<scr_msgs::msg::Log>("/scr/logging", 20, [](const scr_msgs::msg::Log::SharedPtr msg) {
			if (msg->data.length() >= 3 && (msg->data.substr(0, 3) == "50," || msg->data.substr(0, 3) == "20," || msg->data.substr(0, 3) == "51,"))
			{
				return;
			}

			if (msg->node == "autonav_serial_imu")
			{
				return;
			}

			logs.push_back(*msg);
			if (logs.size() > 100)
			{
				logs.erase(logs.begin());
			}
		});
	}

	if (ImGui::Button("Clear Logs"))
	{
		logs.clear();
	}
	ImGui::SameLine();
	if (ImGui::Button("Test Log"))
	{
		node->log("Hello SCR :)");
	}

	ImGui::SeparatorText("Logs");
	for (auto &log : logs)
	{
		ImGui::Text("%s -> %s", log.node.c_str(), log.data.c_str());
	}
}