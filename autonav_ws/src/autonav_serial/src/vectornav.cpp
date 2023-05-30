#include "scr_core/node.h"
#include "rclcpp/rclcpp.hpp"
#include "autonav_msgs/msg/gps_feedback.hpp"
#include "autonav_msgs/msg/imu_data.hpp"

#include "vn/sensors.h"
#include "vn/thread.h"

namespace VectornavConstants
{
    const std::string SENSOR_PORT = "/dev/autonav-imu-200";
    const uint32_t SENSOR_BAUDRATE = 115200;
}

class VectornavNode : public SCR::Node
{
public:
    VectornavNode() : SCR::Node("autonav_vectornav") {}
    ~VectornavNode() {}

    void configure() override
    {
        this->gpsFeedbackPublisher = this->create_publisher<autonav_msgs::msg::GPSFeedback>("/autonav/gps", 20);
        this->imuDataPublisher = this->create_publisher<autonav_msgs::msg::IMUData>("/autonav/imu", 20);
        auto timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VectornavNode::onTimerTick, this));

        sensor.connect(VectornavConstants::SENSOR_PORT, VectornavConstants::SENSOR_BAUDRATE);
        auto modelNumber = sensor.readModelNumber();
        auto firmwareVersion = sensor.readFirmwareVersion();
        auto serialNumber = sensor.readSerialNumber();
        auto hardwareRevision = sensor.readHardwareRevision();

        this->log("Connected to Vectornav IMU");
        this->log("Model Number: " + modelNumber);
        this->log("Firmware Version: " + firmwareVersion);
        this->log("Serial Number: " + serialNumber);
        this->log("Hardware Revision: " + hardwareRevision);

        this->setDeviceState(SCR::DeviceState::OPERATING);
    }

    void onTimerTick()
    {
        if (this->getDeviceState() != SCR::DeviceState::OPERATING)
        {
            return;
        }

        auto gps = sensor.readGpsSolutionLla();
        auto imu = sensor.readImuMeasurements();
        auto ypr = sensor.readYawPitchRoll();

        autonav_msgs::msg::GPSFeedback gpsFeedback;
        gpsFeedback.latitude = gps.lla.x;
        gpsFeedback.longitude = gps.lla.y;
        gpsFeedback.altitude = gps.lla.z;
        gpsFeedback.satellites = gps.numSats;
        gpsFeedback.gps_fix = gps.gpsFix;
        gpsFeedback.is_locked = gps.gpsFix == 0 ? false : true;

        autonav_msgs::msg::IMUData imuData;
        imuData.accel_x = imu.accel.x;
        imuData.accel_y = imu.accel.y;
        imuData.accel_z = imu.accel.z;
        imuData.angular_x = imu.gyro.x;
        imuData.angular_y = imu.gyro.y;
        imuData.angular_z = imu.gyro.z;
        imuData.yaw = ypr.x;
        imuData.pitch = ypr.y;
        imuData.roll = ypr.z;

        this->gpsFeedbackPublisher->publish(gpsFeedback);
        this->imuDataPublisher->publish(imuData);
    }

private:
    vn::sensors::VnSensor sensor;

    rclcpp::Publisher<autonav_msgs::msg::GPSFeedback>::SharedPtr gpsFeedbackPublisher;
    rclcpp::Publisher<autonav_msgs::msg::IMUData>::SharedPtr imuDataPublisher;
};

int main(int a, char** b)
{
    rclcpp::init(a, b);
    rclcpp::spin(std::make_shared<VectornavNode>());
    rclcpp::shutdown();
    return 0;
}