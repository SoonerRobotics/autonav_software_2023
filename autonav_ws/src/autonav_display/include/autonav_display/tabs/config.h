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

void ShowPresetDropdown(SCR::Configuration *config)
{
    auto presets = config->getPresets();
    if (ImGui::BeginCombo("Preset", config->getActivePreset().c_str()))
    {
        for (int i = 0; i < presets.size(); i++)
        {
            bool is_selected = (config->getActivePreset() == presets.at(i));
            if (ImGui::Selectable(presets[i].c_str(), is_selected))
            {
                config->load(presets.at(i));
            }
            if (is_selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
}

void ShowSaveAsPrompt(SCR::Configuration *config)
{
    static char buf[32] = "";
    ImGui::InputText("Name", buf, IM_ARRAYSIZE(buf), ImGuiInputTextFlags_CharsNoBlank);
    if (ImGui::Button("Save") && strlen(buf) > 0)
    {
        config->save(buf);
        ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel"))
    {
        ImGui::CloseCurrentPopup();
    }
}

void ShowPresetActions(SCR::Configuration *config)
{
    if (ImGui::Button("Save"))
    {
        config->save(config->getActivePreset());
    }
    ImGui::SameLine();
    if (ImGui::Button("Save As"))
    {
        ImGui::OpenPopup("Save As");
    }
    ImGui::SameLine();
    if (config->getActivePreset() == "default")
    {
        ImGui::BeginDisabled();
        if (ImGui::Button("Delete Active"))
        {
        }
        ImGui::EndDisabled();
    } else {
        if (ImGui::Button("Delete Active"))
        {
            config->deleteActivePreset();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete All"))
    {
        config->deleteAllPresets();
    }
}

void ShowVisionConfig(SCR::Configuration *config)
{
    if (ImGui::BeginPopupModal("Save As", NULL, ImGuiWindowFlags_AlwaysAutoResize))
    {
        ShowSaveAsPrompt(config);
        ImGui::EndPopup();
    }

    ShowPresetDropdown(config);
    ShowPresetActions(config);

    auto hash = SCR::hash("autonav_nav_resolver");
    if (config->hasDevice(hash))
    {
        ImGui::SeparatorText("Path Resolver");
        ShowFloatOption(config, "Forward Speed", hash, 0, 0, 2);
        ShowFloatOption(config, "Reverse Speed", hash, 1, -2, 2);
        ShowFloatOption(config, "Radius Multiplier", hash, 2, 0, 3);
        ShowFloatOption(config, "Radius Offset", hash, 3, 0, 3);
        ShowFloatOption(config, "Radius Max", hash, 4, 0, 3);
    }

    hash = SCR::hash("autonav_serial_camera");
    if (config->hasDevice(hash))
    {
        ImGui::SeparatorText("Camera");
        ShowIntOption(config, "REFRESH_RATE", hash, 0, 1, 60);
    }

    hash = SCR::hash("autonav_serial_imu");
    if (config->hasDevice(hash))
    {
        ImGui::SeparatorText("IMU");
        ShowFloatOption(config, "Read Rate", hash, 0, 0, 100);
        ShowFloatOption(config, "No IMU Timeout", hash, 1, 1, 60);
        ShowFloatOption(config, "Bad Connection Timeout", hash, 2, 1, 60);
    }

    hash = SCR::hash("autonav_vision_transformer");
    if (config->hasDevice(hash))
    {
        ImGui::SeparatorText("Vision Transformations");
        ShowIntOption(config, "Hue Lower", hash, 0, 0, 255);
        ShowIntOption(config, "Saturation Lower", hash, 1, 0, 255);
        ShowIntOption(config, "Value Lower", hash, 2, 0, 255);
        float h, s, v;
        ImGui::ColorConvertHSVtoRGB(config->get<int>(hash, 0) / 255.0f, config->get<int>(hash, 1) / 255.0f, config->get<int>(hash, 2) / 255.0f, h, s, v);
        ImGui::ColorButton("##HueLower", ImVec4(h, s, v, 1.0f));
        
        ShowIntOption(config, "Hue Upper", hash, 3, 0, 255);
        ShowIntOption(config, "Saturation Upper", hash, 4, 0, 255);
        ShowIntOption(config, "Value Upper", hash, 5, 0, 255);
        ImGui::ColorConvertHSVtoRGB(config->get<int>(hash, 3) / 255.0f, config->get<int>(hash, 4) / 255.0f, config->get<int>(hash, 5) / 255.0f, h, s, v);
        ImGui::ColorButton("##HueUpper", ImVec4(h, s, v, 1.0f));

        ShowIntOption(config, "Blur Amount", hash, 6, 0, 30);
        ShowIntOption(config, "Blur Iterations", hash, 7, 0, 30);
        ShowIntOption(config, "Trapezoid TL", hash, 8, 0, 1000);
        ShowIntOption(config, "Trapezoid TR", hash, 9, 0, 1000);
    }

    hash = SCR::hash("autonav_manual_steamcontroller");
    if (config->hasDevice(hash))
    {
        ImGui::SeparatorText("Manual Control (Steam)");
        ShowFloatOption(config, "Steering Deadzone", hash, 1, 0, 1);
        ShowFloatOption(config, "Throttle Deadzone", hash, 2, 0, 1);
        ShowFloatOption(config, "Forward Speed (m/s)", hash, 3, 0, 5);
        ShowFloatOption(config, "Turn Speed (rad/s)", hash, 4, 0, 5);
    }
}

void ShowConfiguration(SCR::Node *node)
{
    if(!node->config.hasLoadedPreset())
    {
        node->config.load("default");
        return;
    }

    ShowVisionConfig(&node->config);
}