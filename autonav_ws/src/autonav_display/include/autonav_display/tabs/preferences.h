#include "scr_core/node.h"
#include "imgui.h"

void ShowPreferences(SCR::Node *node, float *font_size)
{
	if (ImGui::SliderFloat("Font Size", font_size, 8.0f, 40.0f, "%.1f"))
	{
		ImGui::GetIO().FontGlobalScale = *font_size / 16.0f;
	}
}