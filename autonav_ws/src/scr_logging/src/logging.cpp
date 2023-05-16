#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "scr_core/node.h"
#include "scr_msgs/msg/log.hpp"

#define UNUSED(x) (void)(x)
using std::placeholders::_1;

namespace Constants
{
	std::string LOG_PATH = "/home/{user}/.scr/logs/";
	std::string LOG_FILE_EXT = ".log";
}

long startup_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
std::string savedPath = "";

std::string get_log_path()
{
	if (savedPath != "")
	{
		return savedPath;
	}

	std::string path = Constants::LOG_PATH;
	std::string::size_type i = path.find("{user}");
	if (i != std::string::npos)
	{
		path.replace(i, 6, getenv("USER"));
	}

	savedPath = path;
	return path;
}

void ensure_directory_exists(const std::string &path)
{
	std::string command = "mkdir -p " + path;
	system(command.c_str());
}

void ensure_file_exists(const std::string &file)
{
	std::ofstream out(file, std::ios::app);
	out.close();
}

const std::string generateFilePath(const std::string &node)
{
	std::string path = get_log_path() + std::to_string(startup_time);
	ensure_directory_exists(path);
	std::string file = path + "/" + node + Constants::LOG_FILE_EXT;
	ensure_file_exists(file);
	return file;
}

void append_to_file(const std::string &node, const std::string &contents)
{
	std::ofstream out(generateFilePath(node), std::ios::app);
	out << contents << std::endl;
	out.close();
}

class ConfigurationNode : public SCR::Node
{
public:
	ConfigurationNode() : SCR::Node("scr_logging")
	{
		logSubscriber = this->create_subscription<scr_msgs::msg::Log>("/scr/logging", 10, std::bind(&ConfigurationNode::onLogReceived, this, _1));
		logToConsole = this->declare_parameter<bool>("log_to_console", false);
	}

	void configure() override
	{
		setDeviceState(SCR::DeviceState::OPERATING);
	}

	void transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override
	{
		UNUSED(old);
		UNUSED(updated);
	}

private:
	void onLogReceived(const scr_msgs::msg::Log &msg) const
	{
		if (logToConsole)
		{
			RCLCPP_INFO(this->get_logger(), "[%s] %s", msg.node.c_str(), msg.data.c_str());
		}
		
		append_to_file(msg.node, msg.data);
	}

	rclcpp::Subscription<scr_msgs::msg::Log>::SharedPtr logSubscriber;
	bool logToConsole = false;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ConfigurationNode>());
	rclcpp::shutdown();
	return 0;
}