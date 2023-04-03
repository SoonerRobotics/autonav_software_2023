#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autonav_msgs/msg/log.hpp"
#include "autonav_libs/common.h"

using std::placeholders::_1;

namespace AutonavConstants
{
	std::string LOG_PATH = "/home/{user}/logs/";
	std::string LOG_FILE_EXT = ".log";
	std::string TOPIC = "/autonav/logging";
}

long startup_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
std::string savedPath = "";

std::string get_log_path()
{
	if (savedPath != "")
	{
		return savedPath;
	}

	std::string path = AutonavConstants::LOG_PATH;
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

const std::string generateFilePath(const std::string &name)
{
	std::string path = get_log_path() + std::to_string(startup_time);
	ensure_directory_exists(path);
	std::string file = path + "/" + name + AutonavConstants::LOG_FILE_EXT;
	ensure_file_exists(file);
	return file;
}

void append_to_file(const std::string &name, const std::string &contents)
{
	std::ofstream out(generateFilePath(name), std::ios::app);
	out << contents << std::endl;
	out.close();
}

class LoggingNode : public Autonav::ROS::AutoNode
{
public:
	LoggingNode() : AutoNode("autonav_logging")
	{
		m_steamSubscription = this->create_subscription<autonav_msgs::msg::Log>(AutonavConstants::TOPIC, 10, std::bind(&LoggingNode::on_log_received, this, _1));
	}

	void setup() override
	{
		this->setDeviceState(Autonav::State::DeviceState::READY);
		this->setDeviceState(Autonav::State::DeviceState::OPERATING);
	}

private:
	void on_log_received(const autonav_msgs::msg::Log &msg) const
	{
		RCLCPP_INFO(this->get_logger(), "[%s] %s", msg.file.c_str(), msg.data.c_str());
		append_to_file(msg.file, msg.data);
	}
	rclcpp::Subscription<autonav_msgs::msg::Log>::SharedPtr m_steamSubscription;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LoggingNode>());
	rclcpp::shutdown();
	return 0;
}