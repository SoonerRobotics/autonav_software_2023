#include "scr_core/configuration.h"
#include "scr_core/utils.h"
#include "scr_core/node.h"
#include "imgui.h"

void ShowIntOption(SCR::Configuration *config, std::string name, int64_t device, uint8_t address, int min, int max)
{
    int value = config->get<int>(device, address);
    if (ImGui::InputInt(name.c_str(), &value, 1, 10))
    {
        if (value < min)
            value = min;
        if (value > max)
            value = max;
        config->set<int>(device, address, value);
    }
}

void ShowFloatOption(SCR::Configuration *config, std::string name, int64_t device, uint8_t address, float min, float max)
{
    float value = config->get<float>(device, address);
    if (ImGui::InputFloat(name.c_str(), &value))
    {
        if (value < min)
            value = min;
        if (value > max)
            value = max;
        config->set<float>(device, address, value);
    }
}

void ShowBoolOption(SCR::Configuration *config, std::string name, int64_t device, uint8_t address)
{
    bool value = config->get<bool>(device, address);
    if (ImGui::Checkbox(name.c_str(), &value))
    {
        config->set<bool>(device, address, value);
    }
}

void ShowVisionConfig(SCR::Configuration *config)
{
    ImGui::SeparatorText("Vision Transformations");
    auto hash = SCR::hash("autonav_vision_transformer");
    ShowIntOption(config, "Hue Lower", hash, 0, 0, 255);
    ShowIntOption(config, "Saturation Lower", hash, 1, 0, 255);
    ShowIntOption(config, "Value Lower", hash, 2, 0, 255);
    ShowIntOption(config, "Hue Upper", hash, 3, 0, 255);
    ShowIntOption(config, "Saturation Upper", hash, 4, 0, 255);
    ShowIntOption(config, "Value Upper", hash, 5, 0, 255);
    ShowIntOption(config, "Blur A", hash, 6, 0, 10);
    ShowIntOption(config, "Blur B", hash, 7, 0, 10);
    ShowIntOption(config, "Trapezoid TLX", hash, 8, 0, 1000);
    ShowIntOption(config, "Trapezoid TLY", hash, 9, 0, 1000);

    ImGui::SeparatorText("Path Resolver");
    hash = SCR::hash("autonav_nav_resolver");
    ShowFloatOption(config, "Forward Speed", hash, 0, 0, 2);
    ShowFloatOption(config, "Reverse Speed", hash, 1, -2, 2);
    ShowFloatOption(config, "Radius Multiplier", hash, 2, 0, 3);
    ShowFloatOption(config, "Radius Offset", hash, 3, 0, 3);
    ShowFloatOption(config, "Radius Max", hash, 4, 0, 3);

    ImGui::SeparatorText("Camera");
    hash = SCR::hash("autonav_serial_camera");
    ShowIntOption(config, "REFRESH_RATE", hash, 0, 1, 60);

    ImGui::SeparatorText("IMU");
    hash = SCR::hash("autonav_serial_imu");
    ShowFloatOption(config, "Read Rate", hash, 0, 0, 100);
    ShowFloatOption(config, "No IMU Timeout", hash, 1, 1, 60);
    ShowFloatOption(config, "Bad Connection Timeout", hash, 2, 1, 60);

    ImGui::SeparatorText("Vision Transformations");
    hash = SCR::hash("autonav_vision_transformer");
    ShowIntOption(config, "Hue Lower", hash, 0, 0, 255);
    ShowIntOption(config, "Saturation Lower", hash, 1, 0, 255);
    ShowIntOption(config, "Value Lower", hash, 2, 0, 255);
    ShowIntOption(config, "Hue Upper", hash, 3, 0, 255);
    ShowIntOption(config, "Saturation Upper", hash, 4, 0, 255);
    ShowIntOption(config, "Value Upper", hash, 5, 0, 255);
    ShowIntOption(config, "Blur Amount", hash, 6, 0, 30);
    ShowIntOption(config, "Blur Iterations", hash, 7, 0, 30);
    ShowIntOption(config, "Trapezoid TL", hash, 8, 0, 1000);
    ShowIntOption(config, "Trapezoid TR", hash, 9, 0, 1000);

    ImGui::SeparatorText("Manual Control (Steam)");
    hash = SCR::hash("autonav_manual_steamcontroller");
    ShowIntOption(config, "Timeout Delay", hash, 0, 100, 60000);
    ShowFloatOption(config, "Steering Deadzone", hash, 1, 0, 1);
    ShowFloatOption(config, "Throttle Deadzone", hash, 2, 0, 1);
    ShowFloatOption(config, "Max Speed (m/s)", hash, 3, 0, 5);
    ShowFloatOption(config, "Max Turn Rate (rad/s)", hash, 4, 0, 5);
}

void ShowConfiguration(SCR::Node *node)
{
    if(!node->config.hasLoadedPreset())
    {
        node->config.load("default");
    }

    ShowVisionConfig(&node->config);
}