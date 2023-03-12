#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autonav_msgs/msg/log.hpp"
#include "autonav_msgs/msg/motor_feedback.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/imu_data.hpp"
#include "autonav_libs/common.h"

using std::placeholders::_1;

enum Register
{
	MOTOR_FEEDBACK_ENABLED = 0,
	GPS_FEEDBACK_ENABLED = 1,
	IMU_FEEDBACK_ENABLED = 2
};

class LoggingNode : public Autonav::ROS::AutoNode
{
public:
	LoggingNode() : AutoNode(Autonav::Device::LOGGING_COMBINED, "autonav_logging_combined")
	{
		m_loggingPublisher = this->create_publisher<autonav_msgs::msg::Log>("/autonav/logging", 10);
		m_motorFeedbackSubscriber = this->create_subscription<autonav_msgs::msg::MotorFeedback>("/autonav/MotorFeedback", 10, std::bind(&LoggingNode::motorFeedbackCallback, this, _1));
		m_gpsFeedbackSubscriber = this->create_subscription<autonav_msgs::msg::GPSFeedback>("/autonav/gps", 10, std::bind(&LoggingNode::gpsFeedbackCallback, this, _1));
		m_imuFeedbackSubscriber = this->create_subscription<autonav_msgs::msg::IMUData>("/autonav/imu", 10, std::bind(&LoggingNode::imuFeedbackCallback, this, _1));
	}

	void setup() override
	{
		config.write(Register::MOTOR_FEEDBACK_ENABLED, false);
		config.write(Register::GPS_FEEDBACK_ENABLED, false);
		config.write(Register::IMU_FEEDBACK_ENABLED, false);

		this->setDeviceState(Autonav::State::DeviceState::READY);
		this->setDeviceState(Autonav::State::DeviceState::OPERATING);
	}

	std::string generate_data(autonav_msgs::msg::GPSFeedback::SharedPtr gps, autonav_msgs::msg::IMUData::SharedPtr imu, autonav_msgs::msg::MotorFeedback::SharedPtr motor)
	{
		std::string data = "";
		if(gps != nullptr)
		{
			data += std::to_string(gps->latitude) + "," + std::to_string(gps->longitude) + "," + std::to_string(gps->altitude) + std::to_string(gps->altitude) + "," + (gps->is_locked ? "1" : "0");
		} else {
			data += ",,,";
		}

		if(imu != nullptr)
		{
			data += std::to_string(imu->accel_x) + "," + std::to_string(imu->accel_y) + "," + std::to_string(imu->accel_z) + "," + std::to_string(imu->angular_x) + "," + std::to_string(imu->angular_y) + "," + std::to_string(imu->angular_z) + "," + std::to_string(imu->pitch) + "," + std::to_string(imu->roll) + "," + std::to_string(imu->yaw);
		} else {
			data += ",,,,,,,,";
		}

		if(motor != nullptr)
		{
			data += std::to_string(motor->delta_x) + "," + std::to_string(motor->delta_y) + "," + std::to_string(motor->delta_theta);
		} else {
			data += ",,,";
		}

		return data;
	}

	void motorFeedbackCallback(const autonav_msgs::msg::MotorFeedback::SharedPtr msg)
	{
		if(!config.read<bool>(Register::MOTOR_FEEDBACK_ENABLED))
		{
			return;
		}

		autonav_msgs::msg::Log log;
		log.file = "logging_combined";
		log.data = generate_data(nullptr, nullptr, msg);
		m_loggingPublisher->publish(log);
	}

	void gpsFeedbackCallback(const autonav_msgs::msg::GPSFeedback::SharedPtr msg)
	{
		if(!config.read<bool>(Register::GPS_FEEDBACK_ENABLED))
		{
			return;
		}

		autonav_msgs::msg::Log log;
		log.file = "logging_combined";
		log.data = generate_data(msg, nullptr, nullptr);
		m_loggingPublisher->publish(log);
	}

	void imuFeedbackCallback(const autonav_msgs::msg::IMUData::SharedPtr msg)
	{
		if(!config.read<bool>(Register::IMU_FEEDBACK_ENABLED))
		{
			return;
		}

		autonav_msgs::msg::Log log;
		log.file = "logging_combined";
		log.data = generate_data(nullptr, msg, nullptr);
		m_loggingPublisher->publish(log);
	}

private:
	rclcpp::Publisher<autonav_msgs::msg::Log>::SharedPtr m_loggingPublisher;
	rclcpp::Subscription<autonav_msgs::msg::MotorFeedback>::SharedPtr m_motorFeedbackSubscriber;
	rclcpp::Subscription<autonav_msgs::msg::GPSFeedback>::SharedPtr m_gpsFeedbackSubscriber;
	rclcpp::Subscription<autonav_msgs::msg::IMUData>::SharedPtr m_imuFeedbackSubscriber;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LoggingNode>());
	rclcpp::shutdown();
	return 0;
}