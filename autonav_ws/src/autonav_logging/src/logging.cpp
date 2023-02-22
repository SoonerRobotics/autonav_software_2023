#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autonav_msgs/msg/log.hpp"

using std::placeholders::_1;

namespace AutonavConstants
{
	std::string LOG_PATH = "/home/autonav/logs/";
	std::string LOG_FILE_EXT = ".log";
	std::string TOPIC = "/autonav/logging";
}

long startup_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

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
	// Create a file path that follows the format: /tmp/autonav/logs/<date>/<name>.log
	std::string path = AutonavConstants::LOG_PATH + std::to_string(startup_time);
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

class JoySubscriber : public rclcpp::Node
{
public:
	JoySubscriber() : Node("autonav_logging")
	{
		subscription_ = this->create_subscription<autonav_msgs::msg::Log>(AutonavConstants::TOPIC, 10, std::bind(&JoySubscriber::on_log_received, this, _1));
	}

private:
	void on_log_received(const autonav_msgs::msg::Log &msg) const
	{
		RCLCPP_INFO(this->get_logger(), "[%s] %s", msg.file.c_str(), msg.data.c_str());
		append_to_file(msg.file, msg.data);
	}
	rclcpp::Subscription<autonav_msgs::msg::Log>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JoySubscriber>());
	rclcpp::shutdown();
	return 0;
}