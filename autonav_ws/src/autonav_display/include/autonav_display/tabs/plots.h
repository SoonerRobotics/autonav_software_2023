#include "autonav_msgs/msg/motor_controller_debug.hpp"
#include "scr_core/node.h"
#include "imgui.h"
#include "implot.h"

void ShowPlots(SCR::Node *node)
{
    // Create a subscriber to /autonav/MotorControllerDebug
    static rclcpp::Subscription<autonav_msgs::msg::MotorControllerDebug>::SharedPtr debugSubscriber = nullptr;
    static std::vector<float> times;
    static std::vector<float> forwardVelocities;
    static std::vector<float> forwardSetpoints;
    static std::vector<float> angularVelocities;
    static std::vector<float> angularSetpoints;
    static std::vector<float> leftMotorVelocities;
    static std::vector<float> rightMotorVelocities;

    static bool paused = false;
    static bool showForwardVelocity = true;
    static bool showForwardSetpoint = true;
    static bool showAngularVelocity = true;
    static bool showAngularSetpoint = true;
    static bool showLeftMotorVelocity = true;
    static bool showRightMotorVelocity = true;

    ImGui::Checkbox("Paused", &paused);
    ImGui::Checkbox("Show Forward Velocity", &showForwardVelocity); ImGui::SameLine(); ImGui::Checkbox("Show Forward Setpoint", &showForwardSetpoint);
    ImGui::Checkbox("Show Angular Velocity", &showAngularVelocity); ImGui::SameLine(); ImGui::Checkbox("Show Angular Setpoint", &showAngularSetpoint);
    ImGui::Checkbox("Show Left Motor Velocity", &showLeftMotorVelocity); ImGui::SameLine(); ImGui::Checkbox("Show Right Motor Velocity", &showRightMotorVelocity);

    if (ImGui::Button("Reset Graph"))
    {
        times.clear();
        forwardVelocities.clear();
        forwardSetpoints.clear();
        angularVelocities.clear();
        angularSetpoints.clear();
        leftMotorVelocities.clear();
        rightMotorVelocities.clear();
    }

    if (debugSubscriber == nullptr)
    {
        debugSubscriber = node->create_subscription<autonav_msgs::msg::MotorControllerDebug>(
            "/autonav/MotorControllerDebug",
            20,
            [&](const autonav_msgs::msg::MotorControllerDebug::SharedPtr msg) {
                if (paused)
                {
                    return;
                }

                times.push_back(msg->timestamp / 1000.0f);
                forwardVelocities.push_back(msg->current_forward_velocity);
                forwardSetpoints.push_back(msg->forward_velocity_setpoint);
                angularVelocities.push_back(msg->current_angular_velocity);
                angularSetpoints.push_back(msg->angular_velocity_setpoint);
                leftMotorVelocities.push_back(msg->left_motor_output);
                rightMotorVelocities.push_back(msg->right_motor_output);

                if (times.size() > 700)
                {
                    times.erase(times.begin());
                    forwardVelocities.erase(forwardVelocities.begin());
                    forwardSetpoints.erase(forwardSetpoints.begin());
                    angularVelocities.erase(angularVelocities.begin());
                    angularSetpoints.erase(angularSetpoints.begin());
                    leftMotorVelocities.erase(leftMotorVelocities.begin());
                    rightMotorVelocities.erase(rightMotorVelocities.begin());
                }
            });
    }

    if (times.size() >= 2)
    {
        if (ImPlot::BeginPlot("##Velocities", ImVec2(600, 400)))
        {
            ImPlot::SetupAxes("Time [s]", "Velocity [m/s]", ImPlotAxisFlags_AutoFit, 0);
            ImPlot::SetupLegend(ImPlotLocation_NorthWest);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -1.5, 1.5, ImGuiCond_Always);

            if (showForwardVelocity)
            {
                ImPlot::PlotLine("Forward Velocity", &times[0], &forwardVelocities[0], times.size(), 0, sizeof(float));
            }
            if (showForwardSetpoint)
            {
                ImPlot::PlotLine("Forward Setpoint", &times[0], &forwardSetpoints[0], times.size(), 0, sizeof(float));
            }
            if (showAngularVelocity)
            {
                ImPlot::PlotLine("Angular Velocity", &times[0], &angularVelocities[0], times.size(), 0, sizeof(float));
            }
            if (showAngularSetpoint)
            {
                ImPlot::PlotLine("Angular Setpoint", &times[0], &angularSetpoints[0], times.size(), 0, sizeof(float));
            }
            if (showLeftMotorVelocity)
            {
                ImPlot::PlotLine("Left Motor Velocity", &times[0], &leftMotorVelocities[0], times.size(), 0, sizeof(float));
            }
            if (showRightMotorVelocity)
            {
                ImPlot::PlotLine("Right Motor Velocity", &times[0], &rightMotorVelocities[0], times.size(), 0, sizeof(float));
            }
            ImPlot::EndPlot();
        }


    }
}