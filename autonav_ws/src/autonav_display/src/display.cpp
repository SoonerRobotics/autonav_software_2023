// System Imports
#include <stdio.h>
#include <thread>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>
#include "math.h"

namespace fs = std::filesystem;

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
#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/motor_input.hpp"
#include "autonav_msgs/msg/motor_feedback.hpp"
#include "autonav_msgs/msg/steam_input.hpp"
#include "autonav_msgs/msg/position.hpp"
#include "autonav_msgs/msg/log.hpp"
#include "std_msgs/msg/header.h"
#include "sensor_msgs/msg/image.hpp"

// Other Imports
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.hpp>
#include "opencv4/opencv2/opencv.hpp"

const char *glsl_version = "#version 130";
GLFWwindow *window;

static std::string activePreset = "default";

static void glfw_error_callback(int error, const char *description)
{
	fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

static const char *systemStateToString(Autonav::State::SystemState state)
{
	switch (state)
	{
	case Autonav::State::SystemState::DISABLED:
		return "DISABLED";
	case Autonav::State::SystemState::AUTONOMOUS:
		return "AUTONOMOUS";
	case Autonav::State::SystemState::MANUAL:
		return "MANUAL";
	default:
		return "UNKNOWN";
	}
}

static ImVec4 deviceStateToColor(Autonav::State::DeviceState state)
{
	switch (state)
	{
	case Autonav::State::DeviceState::OFF:
		return ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
	case Autonav::State::DeviceState::STANDBY:
		return ImVec4(1.0f, 1.0f, 0.0f, 1.0f);
	case Autonav::State::DeviceState::READY:
		return ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
	case Autonav::State::DeviceState::OPERATING:
		return ImVec4(0.0f, 1.0f, 1.0f, 1.0f);
	default:
		return ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
	}
}

const char *deviceToString(Autonav::Device device)
{
	switch (device)
	{
	case Autonav::Device::STEAM_TRANSLATOR:
		return "STEAM_TRANSLATOR";
	case Autonav::Device::MANUAL_CONTROL_STEAM:
		return "MANUAL_CONTROL_STEAM";
	case Autonav::Device::MANUAL_CONTROL_XBOX:
		return "MANUAL_CONTROL_XBOX";
	case Autonav::Device::DISPLAY_NODE:
		return "DISPLAY";
	case Autonav::Device::SERIAL_IMU:
		return "IMU_TRANSLATOR";
	case Autonav::Device::SERIAL_CAN:
		return "CAN_TRANSLATOR";
	case Autonav::Device::LOGGING:
		return "LOGGER";
	case Autonav::Device::CAMERA_TRANSLATOR:
		return "CAMERA_TRANSLATOR";
	case Autonav::Device::IMAGE_TRANSFORMER:
		return "IMAGE_TRANSFORMER";
	case Autonav::Device::LOGGING_COMBINED:
		return "LOGGING_COMBINED";
	case Autonav::Device::PARTICLE_FILTER:
		return "FILTERS";
	default:
		return "UNKNOWN";
	}
}

const char *deviceStateToString(Autonav::State::DeviceState state)
{
	switch (state)
	{
	case Autonav::State::DeviceState::STANDBY:
		return "Standby";
	case Autonav::State::DeviceState::READY:
		return "Ready";
	case Autonav::State::DeviceState::OPERATING:
		return "Operating";
	case Autonav::State::DeviceState::OFF:
	default:
		return "OFF";
	}
}

void showIMUConfiguration(Autonav::ROS::AutoNode *node)
{
	auto imu_reg = node->config.getRegistersForDevice(Autonav::Device::SERIAL_IMU);
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
			auto data = node->config.read<float>(Autonav::Device::SERIAL_IMU, address);
			auto title = address == 0 ? "Read Rate (s)" : address == 1 ? "Not Found Retry (s)"
																	   : "Bad Connection Retry (s)";
			if (ImGui::InputFloat(title, &data))
			{
				node->config.writeTo(Autonav::Device::SERIAL_IMU, address, data);
			}
			break;
		}
		}
	}
}

void showManualSteamConfiguration(Autonav::ROS::AutoNode *node)
{
	auto steam_reg = node->config.getRegistersForDevice(Autonav::Device::MANUAL_CONTROL_STEAM);
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
			auto data = node->config.read<int32_t>(Autonav::Device::MANUAL_CONTROL_STEAM, address);
			if (ImGui::InputInt("Timeout Delay", &data))
			{
				node->config.writeTo(Autonav::Device::MANUAL_CONTROL_STEAM, address, data);
			}
			break;
		}

		case 1: // Steering Deadzone
		case 2: // Throttle Deadzone
		case 3: // Max Speed
		case 4: // Speed Offset
		{
			auto data = node->config.read<float>(Autonav::Device::MANUAL_CONTROL_STEAM, address);
			auto title = address == 1 ? "Steering Deadzone" : address == 2 ? "Throttle Deadzone"
														  : address == 3   ? "Max Speed"
																		   : "Speed Offset";
			if (ImGui::InputFloat(title, &data))
			{
				node->config.writeTo(Autonav::Device::MANUAL_CONTROL_STEAM, address, data);
			}
			break;
		}
		}
	}
}

void showCameraConfiguration(Autonav::ROS::AutoNode *node)
{
	auto camera_reg = node->config.getRegistersForDevice(Autonav::Device::CAMERA_TRANSLATOR);
	if (camera_reg.size() == 0)
	{
		return;
	}

	ImGui::SeparatorText("Camera Configuration");
	for (auto it = camera_reg.begin(); it != camera_reg.end(); it++)
	{
		auto address = it->first;
		auto data = it->second;

		switch (address)
		{
		case 0: // Refresh Rate
		{
			auto data = node->config.read<int32_t>(Autonav::Device::CAMERA_TRANSLATOR, address);
			if (ImGui::InputInt("Refresh Rate", &data))
			{
				node->config.writeTo(Autonav::Device::CAMERA_TRANSLATOR, address, data);
			}
			break;
		}
		}
	}
}

void showImageTransformerConfiugration(Autonav::ROS::AutoNode *node)
{
	auto lanemap_reg = node->config.getRegistersForDevice(Autonav::Device::IMAGE_TRANSFORMER);
	if (lanemap_reg.size() == 0)
	{
		return;
	}

	ImGui::SeparatorText("Image Transformer Configuration");
	for (auto it = lanemap_reg.begin(); it != lanemap_reg.end(); it++)
	{
		auto address = it->first;
		auto data = it->second;

		switch (address)
		{
		// 0 through 5 are lower and upper bounds for the color filter in HSV
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		{
			auto title = address == 0 ? "Lower Hue" : address == 1 ? "Lower Sat"
												  : address == 2   ? "Lower Val"
												  : address == 3   ? "Upper Hue"
												  : address == 4   ? "Upper Sat"
																   : "Upper Val";
			auto data = node->config.read<int32_t>(Autonav::Device::IMAGE_TRANSFORMER, address);
			if (ImGui::InputInt(title, &data))
			{
				node->config.writeTo(Autonav::Device::IMAGE_TRANSFORMER, address, data);
			}
			break;
		}

		case 6:
		{
			// Blur Kernel Size
			auto data = node->config.read<int32_t>(Autonav::Device::IMAGE_TRANSFORMER, address);
			if (ImGui::InputInt("Blur Kernel Size", &data))
			{
				node->config.writeTo(Autonav::Device::IMAGE_TRANSFORMER, address, data);
			}
			break;
		}

		case 7:
		{
			// Blur Iterations
			auto data = node->config.read<int32_t>(Autonav::Device::IMAGE_TRANSFORMER, address);
			if (ImGui::InputInt("Blur Iterations", &data))
			{
				node->config.writeTo(Autonav::Device::IMAGE_TRANSFORMER, address, data);
			}
			break;
		}
		}
	}
}

void showCombinedLoggingConfiguration(Autonav::ROS::AutoNode *node)
{
	auto lcomb_reg = node->config.getRegistersForDevice(Autonav::Device::LOGGING_COMBINED);
	if (lcomb_reg.size() == 0)
	{
		return;
	}

	ImGui::SeparatorText("Combined Logger");
	for (auto it = lcomb_reg.begin(); it != lcomb_reg.end(); it++)
	{
		auto address = it->first;
		auto data = it->second;

		switch (address)
		{
		// 0 through 2 are boolean values
		case 0:
		case 1:
		case 2:
		{
			auto title = address == 0 ? "Motor Feedback" : address == 1 ? "GPS" : "IMU";
			auto data = node->config.read<bool>(Autonav::Device::LOGGING_COMBINED, address);
			if (ImGui::Checkbox(title, &data))
			{
				node->config.writeTo(Autonav::Device::LOGGING_COMBINED, address, data);
			}
			break;
		}
		}
	}
}

void showFiltersConfiguration(Autonav::ROS::AutoNode* node)
{
	auto lcomb_reg = node->config.getRegistersForDevice(Autonav::Device::PARTICLE_FILTER);
	if (lcomb_reg.size() == 0)
	{
		return;
	}

	ImGui::SeparatorText("Filters");
	for (auto it = lcomb_reg.begin(); it != lcomb_reg.end(); it++)
	{
		auto address = it->first;
		auto data = it->second;

		switch (address)
		{
			case 0: // A dropdown box of available filters (particle filter, deadreckoning)
			{
				auto data = node->config.read<int32_t>(Autonav::Device::PARTICLE_FILTER, address);
				if (ImGui::Combo("Filter", &data, "Dead Reckoning\0Particle Filter\0"))
				{
					node->config.writeTo(Autonav::Device::PARTICLE_FILTER, address, data);
				}
				break;
			}
			case 1:
			{
				auto data = node->config.read<bool>(Autonav::Device::PARTICLE_FILTER, address);
				if (ImGui::Checkbox("Show Debug Plots", &data))
				{
					node->config.writeTo(Autonav::Device::PARTICLE_FILTER, address, data);
				}
				break;
			}
		}
	}
}

void savePreset(Autonav::ROS::AutoNode *node, std::string file_name)
{
	std::string path = std::string(getenv("HOME")) + "/.config/autonav/configs/" + file_name + ".csv";
	std::ofstream file_stream(path);
	for (auto reg = node->config.begin(); reg != node->config.end(); reg++)
	{
		auto device = reg->first;
		auto entries = reg->second;
		for (auto entry = entries.begin(); entry != entries.end(); entry++)
		{
			auto address = entry->first;
			auto data = entry->second;
			std::string byteString = "";
			for (int i = 0; i < data.size(); i++)
			{
				byteString += std::to_string(data[i]);
				if (i != data.size() - 1)
				{
					byteString += ":";
				}
			}
			file_stream << std::to_string(device) << "," << std::to_string(address) << "," << byteString << std::endl;
		}
	}

	RCLCPP_INFO(node->get_logger(), "Saved configuration to %s", path.c_str());

	file_stream.close();
}

void loadPreset(Autonav::ROS::AutoNode *node, std::string file_name)
{
	std::string path = std::string(getenv("HOME")) + "/.config/autonav/configs/" + file_name + ".csv";
	std::ifstream file_stream(path);
	std::string line;
	std::vector<std::string> lines;
	while (std::getline(file_stream, line))
	{
		lines.push_back(line);
	}

	// Parse each line
	for (auto line : lines)
	{
		std::vector<std::string> bits;
		// Split on commas without boost
		std::stringstream ss(line);
		std::string bit;
		while (std::getline(ss, bit, ','))
		{
			bits.push_back(bit);
		}

		uint8_t device = std::stoi(bits[0]);
		uint8_t address = std::stoi(bits[1]);
		std::vector<uint8_t> data;
		// Split on colons without boost
		std::stringstream ss2(bits[2]);
		std::string bit2;
		while (std::getline(ss2, bit2, ':'))
		{
			data.push_back(std::stoi(bit2));
		}

		auto casted_device = static_cast<Autonav::Device>(device);
		node->config.writeTo(casted_device, address, data);
		RCLCPP_INFO(node->get_logger(), "Writing %d bytes to device %d, address %d", data.size(), device, address);
	}

	RCLCPP_INFO(node->get_logger(), "Loaded configuration from %s", path.c_str());
	activePreset = file_name;
}

std::string removeFileExtension(const std::string file)
{
	std::string::size_type idx;
	idx = file.rfind('.');
	if (idx != std::string::npos)
	{
		return file.substr(0, idx);
	}
	return file;
}

void showConfigPresetDropdown(Autonav::ROS::AutoNode *node)
{
	std::string path = std::string(getenv("HOME")) + "/.config/autonav/configs";
	if (!fs::exists(path))
	{
		fs::create_directories(path);
	}

	// Create a list of all the files in the directory
	std::vector<std::string> files;
	for (const auto &entry : fs::directory_iterator(path))
	{
		files.push_back(entry.path().filename().string());
	}

	if (files.size() == 0)
	{
		savePreset(node, "default");
		files.push_back("default");
		loadPreset(node, "default");
	}

	// // Create a dropdown menu with the list of files
	static int current_item = 0;
	if (ImGui::BeginCombo("Presets", files[current_item].c_str()))
	{
		for (int n = 0; n < files.size(); n++)
		{
			bool is_selected = (current_item == n);
			if (ImGui::Selectable(removeFileExtension(files[n]).c_str(), is_selected))
			{
				loadPreset(node, removeFileExtension(files[n]));
				current_item = n;
			}

			if (is_selected)
			{
				ImGui::SetItemDefaultFocus();
			}
		}
		ImGui::EndCombo();
	}

	// // Create save button
	if (current_item != 0)
	{
		if (ImGui::Button("Save"))
		{
			savePreset(node, files[current_item]);
		}

		ImGui::SameLine();

		if (ImGui::Button("Delete"))
		{
		}
	}
	else
	{
		ImGui::BeginDisabled();
		if (ImGui::Button("Save"))
		{
		}
		ImGui::SameLine();
		if (ImGui::Button("Delete"))
		{
		}
		ImGui::EndDisabled();
	}

	ImGui::SameLine();

	if (ImGui::Button("Save As"))
	{
		ImGui::OpenPopup("Save As");
		ImGui::SetNextWindowSize(ImVec2(300, 100), ImGuiCond_Always);
	}

	if (ImGui::BeginPopupModal("Save As", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize))
	{
		static char buf[128] = "";
		ImGui::InputText("Name", buf, IM_ARRAYSIZE(buf));
		if (ImGui::Button("Save"))
		{
			savePreset(node, buf);
			loadPreset(node, buf);
			ImGui::CloseCurrentPopup();
		}
		ImGui::SameLine();
		if (ImGui::Button("Cancel"))
		{
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
}

class DisplayNode : public Autonav::ROS::AutoNode
{
public:
	DisplayNode() : Autonav::ROS::AutoNode(Autonav::Device::DISPLAY_NODE, "autonav_display") {}

	~DisplayNode()
	{
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImPlot::DestroyContext();
		ImGui::DestroyContext();

		glfwDestroyWindow(window);
		glfwTerminate();
	}

	void setup() override
	{
		RCLCPP_INFO(this->get_logger(), "Starting Display Node");

		m_gpsSubscriber = this->create_subscription<autonav_msgs::msg::GPSFeedback>("/autonav/gps", 20, std::bind(&DisplayNode::onGPSFeedbackReceived, this, std::placeholders::_1));
		m_imuSubscriber = this->create_subscription<autonav_msgs::msg::IMUData>("/autonav/imu", 20, std::bind(&DisplayNode::onImuDataReceived, this, std::placeholders::_1));
		m_motorSubscriber = this->create_subscription<autonav_msgs::msg::MotorInput>("/autonav/MotorInput", 20, std::bind(&DisplayNode::onMotorDataReceived, this, std::placeholders::_1));
		m_motorFeedbackSubscriber = this->create_subscription<autonav_msgs::msg::MotorFeedback>("/autonav/MotorFeedback", 20, std::bind(&DisplayNode::onMotorFeedbackReceived, this, std::placeholders::_1));
		m_steamSubscriber = this->create_subscription<autonav_msgs::msg::SteamInput>("/autonav/joy/steam", 20, std::bind(&DisplayNode::onSteamDataReceived, this, std::placeholders::_1));
		m_filteredCameraSubscriber = this->create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/camera/filtered", 20, std::bind(&DisplayNode::onFilteredCameraDataReceived, this, std::placeholders::_1));
		m_rawCameraSubscriber = this->create_subscription<sensor_msgs::msg::CompressedImage>("/igvc/camera/compressed", 20, std::bind(&DisplayNode::onRawCameraDataReceived, this, std::placeholders::_1));
		m_poseSubscriber = this->create_subscription<autonav_msgs::msg::Position>("/autonav/position", 20, std::bind(&DisplayNode::onPoseReceived, this, std::placeholders::_1));
		m_logSubscriber = this->create_subscription<autonav_msgs::msg::Log>("/autonav/logging", 20, std::bind(&DisplayNode::onLogReceived, this, std::placeholders::_1));

		if (!setup_imgui())
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to setup ImGUI");
			return;
		}

		this->declare_parameter("default_preset", "default");
		activePreset = this->get_parameter("default_preset").as_string();

		this->setDeviceState(Autonav::State::DeviceState::READY);
		this->setDeviceState(Autonav::State::DeviceState::OPERATING);
	}

	void operate() override
	{
		RCLCPP_INFO(this->get_logger(), "Operating Display Node");
		m_renderClock = this->create_wall_timer(std::chrono::milliseconds(1000 / 60), std::bind(&DisplayNode::render, this));
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

		const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		window = glfwCreateWindow(mode->width, mode->height, "Autonav 2023 | Weeb Wagon", NULL, NULL);
		if (window == NULL)
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
		ImPlot::CreateContext();
		ImGuiIO &io = ImGui::GetIO();
		io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

		ImGui::StyleColorsDark();

		// Setup Platform/Renderer backends
		ImGui_ImplGlfw_InitForOpenGL(window, true);
		ImGui_ImplOpenGL3_Init(glsl_version);

		// Custom Font
		std::string path = ament_index_cpp::get_package_share_directory(this->get_name());
		auto font = io.Fonts->AddFontFromFileTTF((path + "/fonts/RobotoMono-VariableFont_wght.ttf").c_str(), 20.0f);
		io.FontDefault = font;
		ImGui_ImplOpenGL3_DestroyFontsTexture();
		ImGui_ImplOpenGL3_CreateFontsTexture();

		// glfwSetWindowMonitor(window, glfwGetPrimaryMonitor(), 0, 0, mode->width, mode->height, mode->refreshRate);
		ImGui::SetNextWindowSize(ImVec2(mode->width * 0.5, mode->height * 0.5), ImGuiCond_Once);

		return true;
	}

	void showNodeState(Autonav::ROS::AutoNode *node, Autonav::Device device)
	{
		auto state = getDeviceState(device);
		ImGui::TextColored(deviceStateToColor(state), "%s: %s", deviceToString(device), deviceStateToString(state));
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
			return;
		}

		const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		glfwPollEvents();
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();

		ImGui::NewFrame();
		{
			ImGui::Begin("Autonav 2023 | The Weeb Wagon", nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoSavedSettings);
			ImGui::SetWindowSize(ImVec2(mode->width, mode->height), ImGuiCond_Once);
			ImGui::SetWindowPos(ImVec2(0, 0), ImGuiCond_Once);

			if (ImGui::BeginTabBar("##Tabs", ImGuiTabBarFlags_None))
			{
				if (ImGui::BeginTabItem("General Data"))
				{
					ImGui::Text("System State: %s", systemStateToString(getSystemState()));
					ImGui::Text("Is Simulator: %s", this->m_isSimulator ? "True" : "False");

					if(m_showGpsData)
					{
						ImGui::SeparatorText("GPS Data");
						ImGui::Text("Latitude: %f", m_lastGpsMessage.latitude);
						ImGui::Text("Longitude: %f", m_lastGpsMessage.longitude);
						ImGui::Text("Altitude: %f", m_lastGpsMessage.altitude);
						ImGui::Text("Current Fix: %d", m_lastGpsMessage.gps_fix);
						ImGui::Text("Locked: %d", m_lastGpsMessage.gps_fix > 0 || m_lastGpsMessage.is_locked);
					}

					if(m_showImuData)
					{
						ImGui::SeparatorText("IMU Data");
						ImGui::Text("Pitch: %f", m_lastImuMessage.pitch);
						ImGui::Text("Roll: %f", m_lastImuMessage.roll);
						ImGui::Text("Yaw: %f", m_lastImuMessage.yaw);
						ImGui::Text("Acceleration: (%f, %f, %f)", m_lastImuMessage.accel_x, m_lastImuMessage.accel_y, m_lastImuMessage.accel_z);
						ImGui::Text("Angular Velocity: (%f, %f, %f)", m_lastImuMessage.angular_x, m_lastImuMessage.angular_y, m_lastImuMessage.angular_z);
					}

					if(m_showEstimatedPose)
					{
						ImGui::SeparatorText("Estimated Position");
						ImGui::Text("X: %f", m_lastPose.x);
						ImGui::Text("Y: %f", m_lastPose.y);
						static auto theta_to_degs = [](double theta) -> double {
							return fmod(360.0 + (theta * 180 / M_PI), 360.0);
						};
						ImGui::Text("Theta: %f radians -> %f degrees", m_lastPose.theta, theta_to_degs(m_lastPose.theta));
						ImGui::Text("Latitude: %f", m_lastPose.latitude);
						ImGui::Text("Longitude: %f", m_lastPose.longitude);
					}

					if(m_showMotorData)
					{
						ImGui::SeparatorText("Motor Data");
						ImGui::Text("Left Motor: %.1f", m_lastMotorMessage.left_motor);
						ImGui::Text("Right Motor: %.1f", m_lastMotorMessage.right_motor);
						ImGui::Text("Delta X, Y, Theta = (%f, %f, %f)", m_lastMotorFeedbackMessage.delta_x, m_lastMotorFeedbackMessage.delta_y, m_lastMotorFeedbackMessage.delta_theta);
					}

					ImGui::SeparatorText("Device States");
					showNodeState(this, Autonav::Device::DISPLAY_NODE);
					showNodeState(this, Autonav::Device::LOGGING);
					showNodeState(this, Autonav::Device::LOGGING_COMBINED);
					showNodeState(this, Autonav::Device::MANUAL_CONTROL_STEAM);
					showNodeState(this, Autonav::Device::MANUAL_CONTROL_XBOX);
					showNodeState(this, Autonav::Device::SERIAL_CAN);
					showNodeState(this, Autonav::Device::SERIAL_IMU);
					showNodeState(this, Autonav::Device::STEAM_TRANSLATOR);
					showNodeState(this, Autonav::Device::CAMERA_TRANSLATOR);
					showNodeState(this, Autonav::Device::IMAGE_TRANSFORMER);
					showNodeState(this, Autonav::Device::PARTICLE_FILTER);

					ImGui::EndTabItem();
				}

				if (ImGui::BeginTabItem("Controller"))
				{
					ImGui::SeparatorText("Controller");
					ImGui::Text("Left Joystick: (%.1f, %.1f)", m_lastSteamMessage.lpad_x, m_lastSteamMessage.lpad_y);
					ImGui::Text("Right Joystick: (%.1f, %.1f)", m_lastSteamMessage.rpad_x, m_lastSteamMessage.rpad_y);
					ImGui::Text("Left Trigger: %.1f", m_lastSteamMessage.ltrig);
					ImGui::Text("Right Trigger: %.1f", m_lastSteamMessage.rtrig);
					for (int i = 0; i < m_lastSteamMessage.buttons.size(); i++)
					{
						ImGui::Text("Button %d: %s", i, m_lastSteamMessage.buttons[i] ? "Pressed" : "Released");
					}

					ImGui::EndTabItem();
				}

				if (ImGui::BeginTabItem("Maps & Path Planning"))
				{
					// Show Camera
					ImGui::SeparatorText("Camera");
					if(m_rawCameraTextureId != -1)
					{
						ImGui::Image((void *)(intptr_t)m_rawCameraTextureId, ImVec2(m_rawCameraWidth, m_rawCameraHeight), ImVec2(0, 0), ImVec2(1, 1));
					}		
					if (m_filteredCameraTextureId != -1)
					{
						ImGui::SameLine();
						ImGui::Image((void *)(intptr_t)m_filteredCameraTextureId, ImVec2(m_filteredCameraWidth, m_filteredCameraHeight), ImVec2(0, 0), ImVec2(1, 1));
					}

					ImGui::EndTabItem();
				}

				if (ImGui::BeginTabItem("Logs"))
				{
					ImGui::BeginChild("ScrollingRegion", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()), false, ImGuiWindowFlags_HorizontalScrollbar);
					ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 1));
					for (int i = 0; i < this->m_latestLogs.size(); i++)
					{
						const char *item = this->m_latestLogs[i].c_str();
						ImGui::TextUnformatted(item);
					}
					ImGui::PopStyleVar();
					ImGui::EndChild();

					ImGui::EndTabItem();
				}

				if (ImGui::BeginTabItem("Configuration"))
				{
					showConfigPresetDropdown(this);
					showIMUConfiguration(this);
					showManualSteamConfiguration(this);
					showCameraConfiguration(this);
					showImageTransformerConfiugration(this);
					showFiltersConfiguration(this);
					showCombinedLoggingConfiguration(this);

					ImGui::EndTabItem();
				}

				if (ImGui::BeginTabItem("Preferences"))
				{
					if (ImGui::SliderFloat("Font Size", &m_fontSize, 10.0f, 40.0f))
					{
						ImGuiIO &io = ImGui::GetIO();
						auto font = io.Fonts->Fonts[0];
						font->Scale = m_fontSize / 20.0f;
					}

					static const char *themes[] = {"Dark", "Light", "Classic"};
					static const char *current_theme = "Dark";
					if (ImGui::BeginCombo("Theme", current_theme))
					{
						for (int i = 0; i < IM_ARRAYSIZE(themes); i++)
						{
							bool is_selected = (current_theme == themes[i]);
							if (ImGui::Selectable(themes[i], is_selected))
							{
								current_theme = themes[i];
								if (strcmp(current_theme, "Dark") == 0)
								{
									ImGui::StyleColorsDark();
								}
								else if (strcmp(current_theme, "Light") == 0)
								{
									ImGui::StyleColorsLight();
								}
								else
								{
									ImGui::StyleColorsClassic();
								}
							}
							if (is_selected)
							{
								ImGui::SetItemDefaultFocus();
							}
						}
						ImGui::EndCombo();
					}

					ImGui::SeparatorText("Data");
					ImGui::Checkbox("Show GPS", &m_showGpsData);
					ImGui::Checkbox("Show IMU", &m_showImuData);
					ImGui::Checkbox("Show Estimated Position", &m_showEstimatedPose);
					ImGui::Checkbox("Show Motor Data", &m_showMotorData);

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
	void onGPSFeedbackReceived(const autonav_msgs::msg::GPSFeedback::SharedPtr data)
	{
		this->m_lastGpsMessage = *data;
	}

	void onImuDataReceived(const autonav_msgs::msg::IMUData::SharedPtr data)
	{
		this->m_lastImuMessage = *data;
	}

	void onMotorDataReceived(const autonav_msgs::msg::MotorInput::SharedPtr data)
	{
		this->m_lastMotorMessage = *data;
	}
	
	void onMotorFeedbackReceived(const autonav_msgs::msg::MotorFeedback::SharedPtr data)
	{
		this->m_lastMotorFeedbackMessage = *data;
	}

	void onSteamDataReceived(const autonav_msgs::msg::SteamInput::SharedPtr data)
	{
		this->m_lastSteamMessage = *data;
	}

	void onPoseReceived(const autonav_msgs::msg::Position::SharedPtr data)
	{
		this->m_lastPose = *data;
	}

	void onFilteredCameraDataReceived(const sensor_msgs::msg::CompressedImage::SharedPtr data)
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

		// Update texture
		auto mat = cv_ptr->image;
		auto width = mat.cols;
		auto height = mat.rows;
		if (m_filteredCameraTextureId == -1)
		{
			glGenTextures(1, &m_filteredCameraTextureId);
		}

		m_filteredCameraWidth = width;
		m_filteredCameraHeight = height;

		glBindTexture(GL_TEXTURE_2D, m_filteredCameraTextureId);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, mat.data);
		glGenerateMipmap(GL_TEXTURE_2D);
	}

	void onRawCameraDataReceived(const sensor_msgs::msg::CompressedImage::SharedPtr data)
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

		// Update texture
		auto mat = cv_ptr->image;
		auto width = mat.cols;
		auto height = mat.rows;
		if (m_rawCameraTextureId == -1)
		{
			glGenTextures(1, &m_rawCameraTextureId);
		}

		m_rawCameraWidth = width;
		m_rawCameraHeight = height;

		glBindTexture(GL_TEXTURE_2D, m_rawCameraTextureId);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, mat.data);
		glGenerateMipmap(GL_TEXTURE_2D);
	}

	void onLogReceived(const autonav_msgs::msg::Log::SharedPtr msg)
	{
		m_latestLogs.insert(m_latestLogs.begin(), msg->data);
		if (m_latestLogs.size() > 100)
		{
			m_latestLogs.erase(m_latestLogs.begin() + 100, m_latestLogs.end());
		}
	}

private:
	rclcpp::Subscription<autonav_msgs::msg::ConBusInstruction>::SharedPtr conbus_subscriber_;
	rclcpp::Subscription<autonav_msgs::msg::GPSFeedback>::SharedPtr m_gpsSubscriber;
	rclcpp::Subscription<autonav_msgs::msg::IMUData>::SharedPtr m_imuSubscriber;
	rclcpp::Subscription<autonav_msgs::msg::MotorInput>::SharedPtr m_motorSubscriber;
	rclcpp::Subscription<autonav_msgs::msg::MotorFeedback>::SharedPtr m_motorFeedbackSubscriber;
	rclcpp::Subscription<autonav_msgs::msg::SteamInput>::SharedPtr m_steamSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_filteredCameraSubscriber;
	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr m_rawCameraSubscriber;
	rclcpp::Subscription<autonav_msgs::msg::Log>::SharedPtr m_logSubscriber;
	rclcpp::Subscription<autonav_msgs::msg::Position>::SharedPtr m_poseSubscriber;

	rclcpp::TimerBase::SharedPtr m_renderClock;

	autonav_msgs::msg::GPSFeedback m_lastGpsMessage;
	autonav_msgs::msg::IMUData m_lastImuMessage;
	autonav_msgs::msg::MotorInput m_lastMotorMessage;
	autonav_msgs::msg::MotorFeedback m_lastMotorFeedbackMessage;
	autonav_msgs::msg::SteamInput m_lastSteamMessage;
	autonav_msgs::msg::Position m_lastPose;
	float m_fontSize = 20.0f;
	std::vector<std::string> m_latestLogs;

	// Images
	GLuint m_filteredCameraTextureId = -1;
	int m_filteredCameraWidth, m_filteredCameraHeight;

	GLuint m_rawCameraTextureId = -1;
	int m_rawCameraWidth, m_rawCameraHeight;

	// Preferences
	bool m_showEstimatedPose = true;
	bool m_showMotorData = true;
	bool m_showGpsData = true;
	bool m_showImuData = true;
};

int main(int, char **)
{
	rclcpp::init(0, NULL);

	auto node = std::make_shared<DisplayNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}