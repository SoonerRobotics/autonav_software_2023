#include "scr_core/node.h"
#include "imgui.h"

void ShowPreferences(SCR::Node *node, float *font_size)
{
	if (ImGui::SliderFloat("Font Size", font_size, 8.0f, 40.0f, "%.1f"))
	{
		ImGui::GetIO().FontGlobalScale = *font_size / 16.0f;
	}

	// Create dropdown for theme
	static const char *themes[] = {"Dark", "Light", "Classic"};
	static int theme_index = 1;
	if (ImGui::Combo("Theme", &theme_index, themes, IM_ARRAYSIZE(themes)))
	{
		if (theme_index == 0) {
			ImGui::StyleColorsDark();
		} else if (theme_index == 1)
		{
			ImGui::StyleColorsLight();
		} else if (theme_index == 2)
		{
			ImGui::StyleColorsClassic();
		}
	}
}