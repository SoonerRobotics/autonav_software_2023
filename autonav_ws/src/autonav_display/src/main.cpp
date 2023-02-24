// System Imports
#include <stdio.h>
#include <thread>
#include <memory>
#include <string>

// ImGui Imports
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// ROS Imports
#include "autonav_libs/autonav.h"
#include "rclcpp/rclcpp.hpp"

#include "autonav_msgs/msg/imu_data.hpp"
#include "autonav_msgs/msg/gps_data.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/steam_input.hpp"

const char *glsl_version = "#version 130";
GLFWwindow *window;

static void glfw_error_callback(int error, const char *description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

class DisplayNode : public rclcpp::Node
{
public:
	DisplayNode() : Node("display_node")
	{
		setup_imgui();

		gps_subscriber_ = this->create_subscription<autonav_msgs::msg::GPSData>("/autonav/gps", 20, std::bind(&DisplayNode::on_gps_data, this, std::placeholders::_1));
		imu_subscriber_ = this->create_subscription<autonav_msgs::msg::IMUData>("/autonav/imu", 20, std::bind(&DisplayNode::on_imu_data, this, std::placeholders::_1));
		motor_subscriber_ = this->create_subscription<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 20, std::bind(&DisplayNode::on_motor_data, this, std::placeholders::_1));
		steam_subscriber_ = this->create_subscription<autonav_msgs::msg::SteamInput>("/autonav/joy/steam", 20, std::bind(&DisplayNode::on_steam_data, this, std::placeholders::_1));

		render_clock = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DisplayNode::render, this));
	}

	void setup_imgui()
	{
		glfwSetErrorCallback(glfw_error_callback);
		if (!glfwInit())
		{
			return;
		}

		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
		// glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
		// glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

		const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		window = glfwCreateWindow(mode->width, mode->height, "Autonav 2023 | Weeb Wagon", NULL, NULL);
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

		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImPlot::CreateContext();
		ImGuiIO &io = ImGui::GetIO();
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

		// Increase font size
		io.Fonts->AddFontDefault();
		ImFontConfig config;
		config.SizePixels = 40.0f;
		io.Fonts->AddFontDefault(&config);

		// Setup Dear ImGui style
		ImGui::StyleColorsDark();

		// Setup Platform/Renderer backends
		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init(glsl_version);

		// Set to full screen
		// glfwSetWindowMonitor(window, glfwGetPrimaryMonitor(), 0, 0, mode->width, mode->height, mode->refreshRate);
		ImGui::SetNextWindowSize(ImVec2(mode->width, mode->height), ImGuiCond_Once);
	}

	void render()
	{
		if(glfwWindowShouldClose(window))
		{
			rclcpp::shutdown();
		}

		const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
		glfwPollEvents();

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		{
			ImGui::Begin("Autonav 2023 | The Weeb Wagon", nullptr);

			ImGui::SetWindowSize(ImVec2(mode->width, mode->height), ImGuiCond_Once);
			ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Once);
			// ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

			if (ImGui::BeginTabBar("##Tabs", ImGuiTabBarFlags_None))
			{
				if (ImGui::BeginTabItem("General Data"))
				{
					ImGui::SeparatorText("GPS Data");
					ImGui::Text("Latitude: %f", gps_data.latitude);
					ImGui::Text("Longitude: %f", gps_data.longitude);
					ImGui::Text("Altitude: %f", gps_data.altitude);
					ImGui::Text("Current Fix: %d", gps_data.gps_fix);

					ImGui::SeparatorText("IMU Data");
					ImGui::Text("Pitch: %f", imu_data.pitch);
					ImGui::Text("Roll: %f", imu_data.roll);
					ImGui::Text("Yaw: %f", imu_data.yaw);
					ImGui::Text("Acceleration: (%f, %f, %f)", imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
					ImGui::Text("Angular Velocity: (%f, %f, %f)", imu_data.angular_x, imu_data.angular_y, imu_data.angular_z);
					
					ImGui::SeparatorText("Motor Data");
					ImGui::Text("Left Motor: %.1f", motor_data.left_motor);
					ImGui::Text("Right Motor: %.1f", motor_data.right_motor);

					ImGui::SeparatorText("Controller");
					ImGui::Text("Left Joystick: (%.1f, %.1f)", steam_data.lpad_x, steam_data.lpad_y);
					ImGui::Text("Right Joystick: (%.1f, %.1f)", steam_data.rpad_x, steam_data.rpad_y);
					ImGui::Text("Left Trigger: %.1f", steam_data.ltrig);
					ImGui::Text("Right Trigger: %.1f", steam_data.rtrig);

					ImGui::EndTabItem();
				}
				ImGui::EndTabBar();
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

	void destroy()
	{
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImPlot::DestroyContext();
		ImGui::DestroyContext();

		glfwDestroyWindow(window);
		glfwTerminate();
	}

	// Other Callbacks
	void on_gps_data(const autonav_msgs::msg::GPSData::SharedPtr data)
	{
		this->gps_data = *data;
	}

	void on_imu_data(const autonav_msgs::msg::IMUData::SharedPtr data)
	{
		this->imu_data = *data;
	}

	void on_motor_data(const autonav_msgs::msg::MotorInput::SharedPtr data)
	{
		this->motor_data = *data;
	}

	void on_steam_data(const autonav_msgs::msg::SteamInput::SharedPtr data)
	{
		this->steam_data = *data;
	}

private:
	rclcpp::Subscription<autonav_msgs::msg::ConBusInstruction>::SharedPtr conbus_subscriber_;
	rclcpp::Subscription<autonav_msgs::msg::GPSData>::SharedPtr gps_subscriber_;
	rclcpp::Subscription<autonav_msgs::msg::IMUData>::SharedPtr imu_subscriber_;
	rclcpp::Subscription<autonav_msgs::msg::MotorInput>::SharedPtr motor_subscriber_;
	rclcpp::Subscription<autonav_msgs::msg::SteamInput>::SharedPtr steam_subscriber_;
	rclcpp::TimerBase::SharedPtr render_clock;

private:
	autonav_msgs::msg::GPSData gps_data;
	autonav_msgs::msg::IMUData imu_data;
	autonav_msgs::msg::MotorInput motor_data;
	autonav_msgs::msg::SteamInput steam_data;
};

int main(int, char **)
{
	rclcpp::init(0, NULL);

	auto node = std::make_shared<DisplayNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	node->destroy();
	return 0;
}