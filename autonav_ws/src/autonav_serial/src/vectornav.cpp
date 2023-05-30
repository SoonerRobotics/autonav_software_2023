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

struct PublisherPtrs
{
    rclcpp::Publisher<autonav_msgs::msg::GPSFeedback>::SharedPtr gpsFeedbackPublisher;
    rclcpp::Publisher<autonav_msgs::msg::IMUData>::SharedPtr imuDataPublisher;
    std::shared_ptr<vn::sensors::VnSensor> sensor;
};

static std::shared_ptr<vn::sensors::VnSensor> sensor;

void onAsciiMessageReceived(void *userData, vn::protocol::uart::Packet &p, size_t index)
{
    if (p.type() != vn::protocol::uart::Packet::TYPE_ASCII)
    {
        return;
    }

    auto publisherPtrs = (PublisherPtrs *)userData;

    if (p.determineAsciiAsyncType() == vn::protocol::uart::AsciiAsync::VNGPS)
    {
        double time;
        uint16_t week;
        uint8_t gpsFix;
        uint8_t numSats;
        vn::math::vec3d lla;
        vn::math::vec3f nedVel;
        vn::math::vec3f nedAcc;
        float speedAcc;
        float timeAcc;
        p.parseGpsSolutionLla(
            &time,
            &week,
            &gpsFix,
            &numSats,
            &lla,
            &nedVel,
            &nedAcc,
            &speedAcc,
            &timeAcc);

        autonav_msgs::msg::GPSFeedback gpsFeedback;
        gpsFeedback.latitude = lla.x;
        gpsFeedback.longitude = lla.y;
        gpsFeedback.altitude = lla.z;
        gpsFeedback.satellites = numSats;
        gpsFeedback.gps_fix = gpsFix;
        gpsFeedback.is_locked = gpsFix == 0 ? false : true;
        publisherPtrs->gpsFeedbackPublisher->publish(gpsFeedback);
        return;
    }

    if (p.determineAsciiAsyncType() == vn::protocol::uart::AsciiAsync::VNIMU)
    {
        vn::math::vec3f accel;
        vn::math::vec3f gyro;
        vn::math::vec3f mag;
        float temp;
        float pres;
        p.parseImuMeasurements(
            &accel,
            &gyro,
            &mag,
            &temp,
            &pres);

        vn::math::vec3f ypr = publisherPtrs->sensor->readYawPitchRoll();

        autonav_msgs::msg::IMUData imuData;
        imuData.accel_x = accel.x;
        imuData.accel_y = accel.y;
        imuData.accel_z = accel.z;
        imuData.angular_x = gyro.x;
        imuData.angular_y = gyro.y;
        imuData.angular_z = gyro.z;
        imuData.yaw = ypr.x;
        imuData.pitch = ypr.y;
        imuData.roll = ypr.z;
        publisherPtrs->imuDataPublisher->publish(imuData);
        return;
    }
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

        auto ptrs = new PublisherPtrs();
        ptrs->gpsFeedbackPublisher = this->gpsFeedbackPublisher;
        ptrs->imuDataPublisher = this->imuDataPublisher;
        ptrs->sensor = sensor;

        sensor->writeAsyncDataOutputType(vn::protocol::uart::VNGPS, true);
        sensor->registerAsyncPacketReceivedHandler(ptrs, onAsciiMessageReceived);

        this->setDeviceState(SCR::DeviceState::OPERATING);
    }


    void transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override
    {
        
    }

private:
    rclcpp::Publisher<autonav_msgs::msg::GPSFeedback>::SharedPtr gpsFeedbackPublisher;
    rclcpp::Publisher<autonav_msgs::msg::IMUData>::SharedPtr imuDataPublisher;
};

int main(int a, char **b)
{
    sensor = std::make_shared<vn::sensors::VnSensor>();
    sensor->connect(VectornavConstants::SENSOR_PORT, VectornavConstants::SENSOR_BAUDRATE);

    rclcpp::init(a, b);
    rclcpp::spin(std::make_shared<VectornavNode>());
    rclcpp::shutdown();

    sensor->disconnect();
    return 0;
}