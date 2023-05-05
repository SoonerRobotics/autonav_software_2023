#include "autonav_msgs/msg/motor_feedback.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/imu_data.hpp"
#include "scr_core/device_state.h"
#include "scr_core/utils.h"
#include "scr_core/node.h"
#include "imgui.h"

ImVec4 fromRGBHex(std::string hex, float alpha)
{
    return ImVec4(
        (float)std::stoi(hex.substr(0, 2), nullptr, 16) / 255.0f,
        (float)std::stoi(hex.substr(2, 2), nullptr, 16) / 255.0f,
        (float)std::stoi(hex.substr(4, 2), nullptr, 16) / 255.0f,
        alpha);
}

ImVec4 toStateColor(const SCR::DeviceState &state)
{
    switch (state)
    {
        case SCR::DeviceState::OFF:
            return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
        case SCR::DeviceState::STANDBY:
            return ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
        case SCR::DeviceState::READY:
            return fromRGBHex("005C14", 1.0f);
        case SCR::DeviceState::OPERATING:
            return ImVec4(0.0f, 0.0f, 1.0f, 1.0f);
        default:
            return ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
    }
}

void ShowStates(SCR::Node *node)
{
    ImGui::SeparatorText("States");
    auto nodes = node->get_node_names();
    for (auto it = nodes.begin(); it != nodes.end(); it++)
    {
        if(it->find("autonav") == std::string::npos)
        {
            continue;
        }

        if (it->find("scr_state_system") != std::string::npos)
		{
			continue;
		}
        
        if(it->at(0) == '/')
        {
            it->erase(0, 1);
        }

        auto deviceState = node->getDeviceState(*it);
        ImGui::TextColored(toStateColor(deviceState), "%s: %s", it->c_str(), toString(deviceState).c_str());
    }
}

float thetaToHeading(float theta)
{
    return fmod(360.0 + (theta * 180 / M_PI), 360.0);
}

void ShowDashboard(SCR::Node *node)
{
    static rclcpp::Subscription<autonav_msgs::msg::Position>::SharedPtr positionSubscriber = nullptr;
    static rclcpp::Subscription<autonav_msgs::msg::GPSFeedback>::SharedPtr gpsSubscriber = nullptr;
    static rclcpp::Subscription<autonav_msgs::msg::IMUData>::SharedPtr imuSubscriber = nullptr;
    static rclcpp::Subscription<autonav_msgs::msg::MotorInput>::SharedPtr motorInputSubscriber = nullptr;
    static rclcpp::Subscription<autonav_msgs::msg::MotorFeedback>::SharedPtr motorFeedbackSubscriber = nullptr;
    static autonav_msgs::msg::Position position;
    static autonav_msgs::msg::GPSFeedback gps;
    static autonav_msgs::msg::IMUData imu;
    static autonav_msgs::msg::MotorInput motorInput;
    static autonav_msgs::msg::MotorFeedback motorFeedback;

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

        imuSubscriber = node->create_subscription<autonav_msgs::msg::IMUData>(
            "autonav/imu", 20, [&](const autonav_msgs::msg::IMUData::SharedPtr msg) {
                imu = *msg;
            }
        );

        motorInputSubscriber = node->create_subscription<autonav_msgs::msg::MotorInput>(
            "autonav/MotorInput", 20, [&](const autonav_msgs::msg::MotorInput::SharedPtr msg) {
                motorInput = *msg;
            }
        );

        motorFeedbackSubscriber = node->create_subscription<autonav_msgs::msg::MotorFeedback>(
            "autonav/MotorFeedback", 20, [&](const autonav_msgs::msg::MotorFeedback::SharedPtr msg) {
                motorFeedback = *msg;
            }
        );
    }

    ImGui::Text("State: %s", toString(static_cast<SCR::SystemState>(node->getSystemState().state)).c_str());
    ImGui::Text("Is Simulator: %s", node->getSystemState().is_simulator ? "Yes" : "No");
    ImGui::Text("Mobility: %s", node->getSystemState().mobility ? "Enabled" : "Disabled");
    ImGui::Text("EStop: %s", node->getSystemState().estop ? "Yes" : "No");

    ImGui::SeparatorText("Motors");
    ImGui::Text("Forward/Angular Velocity (%.3f, %.3f)", motorInput.forward_velocity, motorInput.angular_velocity);
    ImGui::Text("Delta X/Y/Theta (%.3f, %.3f, %.3f)", motorFeedback.delta_x, motorFeedback.delta_y, motorFeedback.delta_theta);

    ImGui::SeparatorText("Estimated Position");
    ImGui::Text("Estimated Position: (%.5f, %.5f, %.5f)", position.x, position.y, thetaToHeading(position.theta));
    ImGui::Text("Estimated GPS: (%.8f, %.8f)", position.latitude, position.longitude);
    
    ImGui::SeparatorText("GPS");
    ImGui::Text("GPS: (%.8f, %.8f, %.8f)", gps.latitude, gps.longitude, gps.altitude);
    ImGui::Text("Current Fix: %d", gps.gps_fix);
    ImGui::Text("Is Fixed: %s", (gps.gps_fix > 0 || gps.is_locked > 0 ? "Yes" : "No"));
    ImGui::Text("Satellites: %d", gps.satellites);

    ImGui::SeparatorText("IMU");
    ImGui::Text("Pitch/Roll/Yaw: (%.5f, %.5f, %.5f)", imu.pitch, imu.roll, imu.yaw);
    ImGui::Text("Acceleration: (%.5f, %.5f, %.5f)", imu.accel_x, imu.accel_y, imu.accel_z);
    ImGui::Text("Angular Velocity: (%.5f, %.5f, %.5f)", imu.angular_x, imu.angular_y, imu.angular_z);

    ShowStates(node);
}