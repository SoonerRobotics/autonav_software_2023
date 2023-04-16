#include <stdio.h>
#include <thread>
#include <memory>
#include <string>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/header.h"
#include "sensor_msgs/msg/image.hpp"
#include "autonav_libs/node.h"
#include "autonav_display/all.h"

#define UNUSED(x) (void)(x)

static GLFWwindow *window;
static void glfw_error_callback(int error, const char *description)
{
	fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

class DisplayNode : public Autonav::AutoNode
{
public:
	DisplayNode() : Autonav::AutoNode("autonav_display") {}

	~DisplayNode() {}

	void configure() override
	{
		this->declare_parameter("default_preset", "default");
		this->declare_parameter("fullscreen", false);

		setDeviceState(Autonav::DeviceState::OPERATING);

		// Create render thread
		render_thread = std::thread([this]() { render(); });
	}

	void transition(autonav_msgs::msg::SystemState old, autonav_msgs::msg::SystemState updated) override
	{
		UNUSED(old);
		switch(updated.state)
		{
			case Autonav::SystemState::SHUTDOWN:
			case Autonav::SystemState::DISABLED:
			case Autonav::SystemState::MANUAL:
			case Autonav::SystemState::AUTONOMOUS:
				break;
		}
	}

	bool setup_imgui()
	{
		glfwSetErrorCallback(glfw_error_callback);
		if (!glfwInit())
		{
			return false;
		}

		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

		const char *glsl_version = "#version 130";
		const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		window = glfwCreateWindow(mode->width, mode->height, "Autonav 2023 | The Weeb Wagon", NULL, NULL);
		if (window == nullptr)
		{
			return false;
		}

		glfwMakeContextCurrent(window);
		glfwSwapInterval(1);
		bool err = glewInit() != GLEW_OK;
		if (err)
		{
			fprintf(stderr, "Failed to initialize OpenGL loader!\n");
			return false;
		}

		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO &io = ImGui::GetIO();
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

		ImGui::StyleColorsDark();

		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init(glsl_version);

		std::string path = ament_index_cpp::get_package_share_directory(this->get_name());
		auto font = io.Fonts->AddFontFromFileTTF((path + "/fonts/RobotoMono-VariableFont_wght.ttf").c_str(), 20.0f);
		io.FontDefault = font;
		ImGui_ImplOpenGL3_DestroyFontsTexture();
		ImGui_ImplOpenGL3_CreateFontsTexture();
		return true;
	}

	void render()
	{
		if (!setup_imgui())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to setup ImGUI");
			return;
		}

		const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		while (!glfwWindowShouldClose(window) && rclcpp::ok())
		{
			glfwPollEvents();
			ImGui_ImplOpenGL3_NewFrame();
			ImGui_ImplGlfw_NewFrame();
			ImGui::NewFrame();

			const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoBringToFrontOnFocus;
			ImGui::Begin("Autonav 2023 | The Weeb Wagon", NULL, flags);
			ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Always);
			ImGui::SetWindowSize(ImVec2(mode->width, mode->height), ImGuiCond_Always);

			if (ImGui::BeginTabBar("##primarytabbar", ImGuiTabBarFlags_None))
			{
				if (ImGui::BeginTabItem("Dashboard"))
				{
					ShowDashboard(this);
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("Vision"))
				{
					ShowVision(this);
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("Configuration"))
				{
					ShowConfiguration(this);
					ImGui::EndTabItem();
				}	
				ImGui::EndTabBar();
			}
			
			ImGui::End();
			ImGui::Render();
			int display_w, display_h;
			glfwGetFramebufferSize(window, &display_w, &display_h);
			glViewport(0, 0, display_w, display_h);
			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT);
			ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
			glfwSwapBuffers(window);
		}

		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImPlot::DestroyContext();
		ImGui::DestroyContext();

		glfwDestroyWindow(window);
		glfwTerminate();

		rclcpp::shutdown();
	}

private:
	std::thread render_thread;
};

int main(int, char **)
{
	rclcpp::init(0, NULL);
	auto node = std::make_shared<DisplayNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}