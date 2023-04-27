#include "scr_core/node.h"
#include "imgui.h"

void ShowDebug(SCR::Node *node)
{
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