#include "autonav_msgs/msg/safety_lights.hpp"
#include "scr_core/node.h"
#include "imgui.h"

void SendSafetyLightsPacket(SCR::Node *node, bool isAuto, int color, int mode)
{
    static rclcpp::Publisher<autonav_msgs::msg::SafetyLights>::SharedPtr safetyLightsPublisher = nullptr;

    if (safetyLightsPublisher == nullptr)
    {
        safetyLightsPublisher = node->create_publisher<autonav_msgs::msg::SafetyLights>("/autonav/SafetyLights", 20);
    }

    autonav_msgs::msg::SafetyLights safetyLights;
    safetyLights.autonomous = isAuto;
    safetyLights.color = color;
    safetyLights.preset = mode;
    safetyLightsPublisher->publish(safetyLights);
}

void ShowDebug(SCR::Node *node)
{
    ImGui::SeparatorText("System");
    if (ImGui::Button("Reset"))
    {
        node->reset();
    }

    auto mobilityStr = node->getSystemState().mobility ? "Disable Mobility" : "Enable Mobility";
    if(ImGui::Button(mobilityStr))
    {
        node->setMobility(!node->getSystemState().mobility);
    }

    // Create a dropdown for the different system states
    static int systemStateIndex = 0;
    const char *systemStates[] = {"Disabled", "Autonomous", "Manual", "Shutdown"};
    if (ImGui::BeginCombo("System State", systemStates[systemStateIndex]))
    {
        for (int i = 0; i < IM_ARRAYSIZE(systemStates); i++)
        {
            bool isSelected = (systemStateIndex == i);
            if (ImGui::Selectable(systemStates[i], isSelected))
            {
                systemStateIndex = i;
                node->setSystemState(static_cast<SCR::SystemState>(i));
            }
            if (isSelected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::SeparatorText("Safety Lights");
    static bool autonomous = false;
    static int colorIndex = 0;
    const char *colors[] = {"Blank", "Red", "Orange", "Yellow", "Lime", "Green", "Cyan Green", "Cyan", "Light Blue", "Blue", "Violet", "Purple", "Pink", "White Green", "White Red", "White"};
    static int modeIndex = 0;
    const char *modes[] = {"Solid", "Flash", "Fade", "Rainbow"};

    if (ImGui::Checkbox("Autonomous", &autonomous))
    {
        SendSafetyLightsPacket(node, autonomous, colorIndex, modeIndex);
    }

    if (ImGui::BeginCombo("Color", colors[colorIndex]))
    {
        for (int i = 0; i < IM_ARRAYSIZE(colors); i++)
        {
            bool isSelected = (colorIndex == i);
            if (ImGui::Selectable(colors[i], isSelected))
            {
                colorIndex = i;
                SendSafetyLightsPacket(node, autonomous, colorIndex, modeIndex);
            }
            if (isSelected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    if (ImGui::BeginCombo("Mode", modes[modeIndex]))
    {
        for (int i = 0; i < IM_ARRAYSIZE(modes); i++)
        {
            bool isSelected = (modeIndex == i);
            if (ImGui::Selectable(modes[i], isSelected))
            {
                modeIndex = i;
                SendSafetyLightsPacket(node, autonomous, colorIndex, modeIndex);
            }
            if (isSelected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::SeparatorText("ImGUI");
    static bool showImGuiDemo = false;
    ImGui::Checkbox("Show ImGui Demo", &showImGuiDemo);

    if (showImGuiDemo)
    {
        ImGui::ShowDemoWindow(&showImGuiDemo);
    }

    ImGui::Text("Wants Keyboard Input: %s", ImGui::GetIO().WantCaptureKeyboard ? "true" : "false");
    ImGui::Text("Wants Mouse Input: %s", ImGui::GetIO().WantCaptureMouse ? "true" : "false");
    ImGui::Text("Wants Set Mouse Pos: %s", ImGui::GetIO().WantSetMousePos ? "true" : "false");
    ImGui::Text("Wants TextInput: %s", ImGui::GetIO().WantTextInput ? "true" : "false");
}