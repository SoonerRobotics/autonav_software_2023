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
#include "autonav_libs/common.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "autonav_msgs/msg/imu_data.hpp"
#include "autonav_msgs/msg/gps_data.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/steam_input.hpp"
#include "std_msgs/msg/header.h"
#include "sensor_msgs/msg/image.hpp"

// Other Imports
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.hpp>
#include "opencv4/opencv2/opencv.hpp"

// TODO: Fix image flashing issue

const char *glsl_version = "#version 130";
GLFWwindow *window;

static void glfw_error_callback(int error, const char *description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

static ImVec4 deviceStateToColor(Autonav::State::DeviceState state)
{
	switch (state)
	{
	case Autonav::State::DeviceState::OFF:
		return ImVec4(0.5f, 0.5f, 0.5f, 1.0f);
	case Autonav::State::DeviceState::STANDBY:
		return ImVec4(0.5f, 0.5f, 0.0f, 1.0f);
	case Autonav::State::DeviceState::READY:
		return ImVec4(0.0f, 0.5f, 0.0f, 1.0f);
	case Autonav::State::DeviceState::OPERATING:
		return ImVec4(0.0f, 0.5f, 0.5f, 1.0f);
	default:
		return ImVec4(0.5f, 0.0f, 0.0f, 1.0f);
	}
}

void showSerialConfiguration(Autonav::ROS::AutoNode *node)
{
	auto can_reg = node->_config.getRegistersForDevice(Autonav::Device::SERIAL_CAN);
	if (can_reg.size() == 0)
	{
		return;
	}

	ImGui::SeparatorText("Serial Configuration");
	for (auto it = can_reg.begin(); it != can_reg.end(); it++)
	{
		auto address = it->first;
		auto data = it->second;

		switch (address)
		{
		case 0: // Motor Offset
		{
			auto data = node->_config.read<float>(Autonav::Device::SERIAL_CAN, address);
			if (ImGui::InputFloat("Motor Offset", &data))
			{
				node->_config.writeTo(Autonav::Device::SERIAL_CAN, address, data);
			}
			break;
		}
		}
	}
}

void showIMUConfiguration(Autonav::ROS::AutoNode *node)
{
	auto imu_reg = node->_config.getRegistersForDevice(Autonav::Device::SERIAL_IMU);
	if (imu_reg.size() == 0)
	{
		return;
	}

	ImGui::SeparatorText("IMU Configuration");
	for (auto it = imu_reg.begin(); it != imu_reg.end(); it++)
	{
		auto address = it->first;
		auto data = it->second;

		switch (address)
		{
		case 0: // Read Rate
		case 1: // Not Found Retry Time
		case 2: // Bad Connection Retry
		{
			auto data = node->_config.read<float>(Autonav::Device::SERIAL_IMU, address);
			auto title = address == 0 ? "Read Rate (s)" : address == 1 ? "Not Found Retry (s)"
																	   : "Bad Connection Retry (s)";
			if (ImGui::InputFloat(title, &data))
			{
				node->_config.writeTo(Autonav::Device::SERIAL_IMU, address, data);
			}
			break;
		}
		}
	}
}

void showManualSteamConfiguration(Autonav::ROS::AutoNode *node)
{
	auto steam_reg = node->_config.getRegistersForDevice(Autonav::Device::MANUAL_CONTROL_STEAM);
	if (steam_reg.size() == 0)
	{
		return;
	}

	ImGui::SeparatorText("Steam Controller Configuration");
	for (auto it = steam_reg.begin(); it != steam_reg.end(); it++)
	{
		auto address = it->first;
		auto data = it->second;

		switch (address)
		{
		case 0: // Timeout Delay
		{
			auto data = node->_config.read<int32_t>(Autonav::Device::MANUAL_CONTROL_STEAM, address);
			if (ImGui::InputInt("Timeout Delay", &data))
			{
				node->_config.writeTo(Autonav::Device::MANUAL_CONTROL_STEAM, address, data);
			}
			break;
		}

		case 1: // Steering Deadzone
		case 2: // Throttle Deadzone
		case 3: // Max Speed
		case 4: // Speed Offset
		{
			auto data = node->_config.read<float>(Autonav::Device::MANUAL_CONTROL_STEAM, address);
			auto title = address == 1 ? "Steering Deadzone" : address == 2 ? "Throttle Deadzone"
														  : address == 3   ? "Max Speed"
																		   : "Speed Offset";
			if (ImGui::InputFloat(title, &data))
			{
				node->_config.writeTo(Autonav::Device::MANUAL_CONTROL_STEAM, address, data);
			}
			break;
		}
		}
	}
}

void showNodeState(Autonav::ROS::AutoNode *node, Autonav::Device device)
{
	auto state = node->_deviceStates[device];
	ImGui::TextColored(deviceStateToColor(state), "%s: %s", Autonav::deviceToString(device), Autonav::deviceStateToString(state));
}

class DisplayNode : public Autonav::ROS::AutoNode
{
public:
	DisplayNode() : Autonav::ROS::AutoNode(Autonav::Device::DISPLAY_NODE, "autonav_display")
	{
		setup_imgui();

		gps_subscriber_ = this->create_subscription<autonav_msgs::msg::GPSData>("/autonav/gps", 20, std::bind(&DisplayNode::on_gps_data, this, std::placeholders::_1));
		imu_subscriber_ = this->create_subscription<autonav_msgs::msg::IMUData>("/autonav/imu", 20, std::bind(&DisplayNode::on_imu_data, this, std::placeholders::_1));
		motor_subscriber_ = this->create_subscription<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 20, std::bind(&DisplayNode::on_motor_data, this, std::placeholders::_1));
		steam_subscriber_ = this->create_subscription<autonav_msgs::msg::SteamInput>("/autonav/joy/steam", 20, std::bind(&DisplayNode::on_steam_data, this, std::placeholders::_1));
		camera_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/camera/compressed", 20, std::bind(&DisplayNode::on_camera_data, this, std::placeholders::_1));
		render_clock = this->create_wall_timer(std::chrono::milliseconds(1000 / 60), std::bind(&DisplayNode::render, this));
	}

	~DisplayNode()
	{
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImPlot::DestroyContext();
		ImGui::DestroyContext();

		glfwDestroyWindow(window);
		glfwTerminate();
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

		ImGui::StyleColorsDark();

		// Setup Platform/Renderer backends
		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init(glsl_version);

		// Custom Font
		std::string path = ament_index_cpp::get_package_share_directory(this->get_name());
		auto font = io.Fonts->AddFontFromFileTTF((path + "/fonts/RobotoMono-VariableFont_wght.ttf").c_str(), 24.0f);
		io.FontDefault = font;
		ImGui_ImplOpenGL3_DestroyFontsTexture();
		ImGui_ImplOpenGL3_CreateFontsTexture();

		// Screen Size Adapations
		io.DisplaySize = ImVec2(mode->width, mode->height);
		io.DisplayFramebufferScale = ImVec2(1.0f, 1.0f);

		// glfwSetWindowMonitor(window, glfwGetPrimaryMonitor(), 0, 0, mode->width, mode->height, mode->refreshRate);
		ImGui::SetNextWindowSize(ImVec2(mode->width * 0.5, mode->height * 0.5), ImGuiCond_Once);

		// Send out our device state
		this->setDeviceState(Autonav::State::DeviceState::OPERATING);

		this->_config.requestAllRemoteRegisters();
	}

	void render()
	{
		if (glfwWindowShouldClose(window) || !rclcpp::ok())
		{
			if (rclcpp::ok())
			{
				this->setDeviceState(Autonav::State::DeviceState::OFF);
			}

			rclcpp::shutdown();
		}

		const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
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
					ImGui::Text("System State: %d", _systemState);

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
					for (int i = 0; i < steam_data.buttons.size(); i++)
					{
						ImGui::Text("Button %d: %s", i, steam_data.buttons[i] ? "Pressed" : "Released");
					}

					ImGui::SeparatorText("Device States");
					showNodeState(this, Autonav::Device::DISPLAY_NODE);
					showNodeState(this, Autonav::Device::LOGGING);
					showNodeState(this, Autonav::Device::MANUAL_CONTROL_STEAM);
					showNodeState(this, Autonav::Device::MANUAL_CONTROL_XBOX);
					showNodeState(this, Autonav::Device::SERIAL_CAN);
					showNodeState(this, Autonav::Device::SERIAL_IMU);
					showNodeState(this, Autonav::Device::STEAM_TRANSLATOR);

					if (this->has_image)
					{
						GLuint texture;
						cv::cvtColor(image, image, cv::COLOR_BGR2RGBA);
						glGenTextures(1, &texture);
						glBindTexture(GL_TEXTURE_2D, texture);
						glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
						glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
						glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
						glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.cols, image.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data);
						ImGui::Image(reinterpret_cast<void *>(static_cast<intptr_t>(texture)), ImVec2(image.cols, image.rows));
					}
					ImGui::EndTabItem();
				}

				if (ImGui::BeginTabItem("Maps & Path Planning"))
				{
					ImGui::EndTabItem();
				}

				if (ImGui::BeginTabItem("Configuration"))
				{
					if (ImGui::Checkbox("Show Raw", &this->show_conbus_raw))
					{
					}

					if (ImGui::Button("Request All"))
					{
						this->_config.requestAllRemoteRegisters();
					}

					showSerialConfiguration(this);
					showIMUConfiguration(this);
					showManualSteamConfiguration(this);

					ImGui::EndTabItem();
				}

				if (ImGui::BeginTabItem("Preferences"))
				{
					ImGui::EndTabItem();
				}

				if (ImGui::BeginTabItem("Debug"))
				{
					char cwd[1024];
					if (getcwd(cwd, sizeof(cwd)) != NULL)
					{
						ImGui::Text("Current working dir: %s", cwd);
					}
					else
					{
						ImGui::Text("Current working dir: %s", "Error getting current working directory");
					}

					std::string path = ament_index_cpp::get_package_share_directory(this->get_name());
					ImGui::Text("Package directory: %s", path.c_str());
					ImGui::EndTabItem();

					// Show a tree of all topics
					if (ImGui::TreeNode("Topics"))
					{
						auto _topics = this->get_topic_names_and_types();
						for (auto topic : _topics)
						{
							ImGui::Text("%s", topic.first.c_str());
							for (auto type : topic.second)
							{
								ImGui::Text("  %s", type.c_str());
							}
						}
						ImGui::TreePop();
					}

					// Show a tree of all nodes
					if (ImGui::TreeNode("Nodes"))
					{
						auto _nodes = this->get_node_names();
						for (auto node : _nodes)
						{
							ImGui::Text("%s", node.c_str());
						}
						ImGui::TreePop();
					}
				}

				ImGui::EndTabBar();
			}
			ImGui::End();
		}

		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		glfwSwapBuffers(window);
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

	void on_camera_data(const sensor_msgs::msg::CompressedImage::SharedPtr data)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(data, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception &e)
		{
			RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
			return;
		}

		this->has_image = true;
		this->image = cv_ptr->image;
	}

private:
	bool show_conbus_raw = false;

private:
	rclcpp::Subscription<autonav_msgs::msg::ConBusInstruction>::SharedPtr conbus_subscriber_;
	rclcpp::Subscription<autonav_msgs::msg::GPSData>::SharedPtr gps_subscriber_;
	rclcpp::Subscription<autonav_msgs::msg::IMUData>::SharedPtr imu_subscriber_;
	rclcpp::Subscription<autonav_msgs::msg::MotorInput>::SharedPtr motor_subscriber_;
	rclcpp::Subscription<autonav_msgs::msg::SteamInput>::SharedPtr steam_subscriber_;
	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscriber_;

	rclcpp::TimerBase::SharedPtr render_clock;

private:
	autonav_msgs::msg::GPSData gps_data;
	autonav_msgs::msg::IMUData imu_data;
	autonav_msgs::msg::MotorInput motor_data;
	autonav_msgs::msg::SteamInput steam_data;
	cv::Mat image;
	bool has_image = false;
};

int main(int, char **)
{
	rclcpp::init(0, NULL);

	auto node = std::make_shared<DisplayNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}