#include "scr_core/node.h"
#include "rclcpp/rclcpp.hpp"

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

private:
    vn::sensors::VnSensor sensor;
};

int main(int a, char** b)
{
    auto sensor = vn::sensors::VnSensor();
    sensor.connect(VectornavConstants::SENSOR_PORT, VectornavConstants::SENSOR_BAUDRATE);
    auto modelNumber = sensor.readModelNumber();
    auto firmwareVersion = sensor.readFirmwareVersion();
    auto serialNumber = sensor.readSerialNumber();
    auto hardwareRevision = sensor.readHardwareRevision();

    auto gps = sensor.readGpsSolutionLla();
    auto imu = sensor.readImuMeasurements();

    std::cout << "Connected to Vectornav IMU" << std::endl;
    std::cout << "Model Number: " << modelNumber << std::endl;
    std::cout << "Firmware Version: " << firmwareVersion << std::endl;
    std::cout << "Serial Number: " << serialNumber << std::endl;
    std::cout << "Hardware Revision: " << hardwareRevision << std::endl;

    std::cout << "GPS: " << "(" << gps.lla.x << "," << gps.lla.y << "," << gps.lla.z << ")" << std::endl;

    std::cout << "Acceleration: " << "(" << imu.accel.x << "," << imu.accel.y << "," << imu.accel.z << ")" << std::endl;
    std::cout << "Angular Rate: " << "(" << imu.gyro.x << "," << imu.gyro.y << "," << imu.gyro.z << ")" << std::endl;
    std::cout << "Magnetic Field: " << "(" << imu.mag.x << "," << imu.mag.y << "," << imu.mag.z << ")" << std::endl;
    std::cout << "Temperature: " << imu.temp << std::endl;
    std::cout << "Pressure: " << imu.pressure << std::endl;

    rclcpp::init(a, b);
    rclcpp::spin(std::make_shared<VectornavNode>());
    rclcpp::shutdown();
    return 0;
}