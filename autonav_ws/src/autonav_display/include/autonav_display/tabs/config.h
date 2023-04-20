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
}

void ShowConfiguration(SCR::Node *node)
{
    ShowVisionConfig(&node->config);
}