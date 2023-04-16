#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_libs/utils.h"
#include "autonav_libs/device_state.h"
#include "autonav_libs/node.h"
#include "imgui.h"

void ShowStates(Autonav::AutoNode *node)
{
    ImGui::SeparatorText("States");
    auto nodes = node->get_node_names();
    for (auto it = nodes.begin(); it != nodes.end(); it++)
    {
        if(it->find("autonav") == std::string::npos)
        {
            continue;
        }

        if (it->find("autonav_state_system") != std::string::npos)
		{
			continue;
		}
        
        if(it->at(0) == '/')
        {
            it->erase(0, 1);
        }

        auto deviceState = node->getDeviceState(*it);
        ImGui::Text("%s: %s", it->c_str(), toString(deviceState).c_str());
    }
}

void ShowDashboard(Autonav::AutoNode *node)
{
    static rclcpp::Subscription<autonav_msgs::msg::Position>::SharedPtr positionSubscriber = nullptr;
    static rclcpp::Subscription<autonav_msgs::msg::GPSFeedback>::SharedPtr gpsSubscriber = nullptr;
    static autonav_msgs::msg::Position position;
    static autonav_msgs::msg::GPSFeedback gps;

    if (positionSubscriber == nullptr)
    {
        positionSubscriber = node->create_subscription<autonav_msgs::msg::Position>(
            "autonav/position", 20, [&](const autonav_msgs::msg::Position::SharedPtr msg) {
                position = *msg;
            }
        );

        gpsSubscriber = node->create_subscription<autonav_msgs::msg::GPSFeedback>(
            "autonav/gps", 20, [&](const autonav_msgs::msg::GPSFeedback::SharedPtr msg) {
                gps = *msg;
            }
        );
    }

    ImGui::Text("Estimated Position: (%.5f, %.5f, %.5f)", position.x, position.y, position.theta);
    ImGui::Text("GPS: (%.5f, %.5f, %.5f)", gps.latitude, gps.longitude, gps.altitude);

    ShowStates(node);
}