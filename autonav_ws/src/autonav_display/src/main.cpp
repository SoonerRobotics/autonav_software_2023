// System Imports
#include <stdio.h>
#include <thread>
#include <memory>
#include <string>

// ImGui Imports
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// ROS Imports
#include "autonav_libs/autonav.h";
#include "rclcpp/rclcpp.hpp"

static std::atomic_uint8_t* config_map[100];
static std::atomic_bool running = true;

static void glfw_error_callback(int error, const char *description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void imgui_thread()
{
	glfwSetErrorCallback(glfw_error_callback);
	if (!glfwInit())
	{
		return;
	}

	const char *glsl_version = "#version 150";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

	const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	GLFWwindow *window = glfwCreateWindow(mode->width, mode->height, "imgui_vendor example", NULL, NULL);
	if (window == NULL)
	{
		return;
	}
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	bool err = glewInit() != GLEW_OK;
	if (err)
	{
		fprintf(stderr, "Failed to initialize OpenGL loader!\n");
		return;
	}

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO &io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	// Set to full screen
	// glfwSetWindowMonitor(window, glfwGetPrimaryMonitor(), 0, 0, mode->width, mode->height, mode->refreshRate);

	ImGui::SetNextWindowSize(ImVec2(mode->width, mode->height), ImGuiCond_Once);

	ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	while (!glfwWindowShouldClose(window) && running)
	{
		glfwPollEvents();

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		{
			ImGui::Begin("Hello, world!", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
			ImGui::Text(
				"Application average %.3f ms/frame (%.1f FPS)",
				1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
				

			// Loop through config_map
			for (int i = 0; i < 100; i++)
			{
				if (config_map[i] != NULL)
				{
					uint8_t* data = (uint8_t*)config_map[i];
					auto addr = data[0];
					// Concat data into a string
					std::string str = "";
					for (int j = 1; j < 1 + data[1]; j++)
					{
						str += std::to_string(data[j]);
						str += " ";
					}

					ImGui::Text("Address: %d, Data: %s", addr, str.c_str());
				}
			}

			ImGui::End();
		}

		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(
			clear_color.x * clear_color.w, clear_color.y * clear_color.w,
			clear_color.z * clear_color.w, clear_color.w);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
	}

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow(window);
	glfwTerminate();
}

class DisplayNode : public rclcpp::Node
{
public:
	DisplayNode() : Node("display_node")
	{
		conbus_subscriber_ = this->create_subscription<autonav_msgs::msg::ConBusInstruction>("/autonav/conbus", 20, std::bind(&DisplayNode::conbus_callback, this, std::placeholders::_1));
	}

private:
	void conbus_callback(const autonav_msgs::msg::ConBusInstruction::SharedPtr msg)
	{
		// Create data field with 1 + data.length() elements
		uint8_t* data = (uint8_t*)malloc(1 + msg->data.size());
		data[0] = msg->address;
		memcpy(data + 1, msg->data.data(), msg->data.size());
		config_map[msg->address] = (std::atomic_uint8_t*)data;
	}

private:
	rclcpp::Subscription<autonav_msgs::msg::ConBusInstruction>::SharedPtr conbus_subscriber_;
};

int main(int, char **)
{
	rclcpp::init(0, NULL);

	std::thread imgui(imgui_thread);

	rclcpp::spin(std::make_shared<DisplayNode>());
	rclcpp::shutdown();
	running = false;
	imgui.join();
	return 0;
}