#include "autonav_msgs/msg/motor_feedback.hpp"
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
}