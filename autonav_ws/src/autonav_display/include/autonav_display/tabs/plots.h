#include "autonav_msgs/msg/motor_controller_debug.hpp"
#include "scr_core/node.h"
#include "imgui.h"
#include "implot.h"

float start_time = 0.0f;

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

    if (debugSubscriber == nullptr)
    {
        debugSubscriber = node->create_subscription<autonav_msgs::msg::MotorControllerDebug>(
            "/autonav/MotorControllerDebug",
            20,
            [&](const autonav_msgs::msg::MotorControllerDebug::SharedPtr msg) {
                times.push_back(msg->timestamp);
                forwardVelocities.push_back(msg->current_forward_velocity);
                forwardSetpoints.push_back(msg->forward_velocity_setpoint);
                angularVelocities.push_back(msg->current_angular_velocity);
                angularSetpoints.push_back(msg->angular_velocity_setpoint);
                leftMotorVelocities.push_back(msg->left_motor_output);
                rightMotorVelocities.push_back(msg->right_motor_output);
            });
    }

    // Create a plot for forward velocity. Plotting the 6 values against timestamp
    if (ImPlot::BeginPlot("Motor Data"))
    {
        ImPlot::PlotLine("Forward Velocity", times.data(), forwardVelocities.data(), times.size());
        ImPlot::PlotLine("Forward Setpoint", times.data(), forwardSetpoints.data(), times.size());
        ImPlot::PlotLine("Angular Velocity", times.data(), angularVelocities.data(), times.size());
        ImPlot::PlotLine("Angular Setpoint", times.data(), angularSetpoints.data(), times.size());
        ImPlot::PlotLine("Left Motor Velocity", times.data(), leftMotorVelocities.data(), times.size());
        ImPlot::PlotLine("Right Motor Velocity", times.data(), rightMotorVelocities.data(), times.size());
        ImPlot::EndPlot();
    }
}