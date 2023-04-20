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

    ImGui::Text("Wants Keyboard Input: %s", ImGui::GetIO().WantCaptureKeyboard ? "true" : "false");
    ImGui::Text("Wants Mouse Input: %s", ImGui::GetIO().WantCaptureMouse ? "true" : "false");
    ImGui::Text("Wants Set Mouse Pos: %s", ImGui::GetIO().WantSetMousePos ? "true" : "false");
    ImGui::Text("Wants TextInput: %s", ImGui::GetIO().WantTextInput ? "true" : "false");
}