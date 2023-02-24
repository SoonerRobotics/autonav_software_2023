#include "autonav_msgs/msg/con_bus_instruction.hpp"
#include "autonav_msgs/msg/device_state.hpp"
#include "autonav_msgs/msg/system_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <stdint.h>
#include <map>
#include <string.h>

namespace Autonav
{
    enum Device : uint8_t
    {
        STEAM_TRANSLATOR = 100,
        MANUAL_CONTROL = 101,
        DISPLAY_NODE = 102,
        SERIAL_IMU = 103,
        SERIAL_CAN = 104,
        LOGGING = 105
    };

    namespace State
    {
        enum DeviceState
        {
            OFF = 1,
            STANDBY = 2,
            READY = 3,
            OPERATING = 4
        };

        enum SystemState
        {
            DISABLED = 1,
            AUTONOMOUS = 2,
            MANUAL = 3
        };
    }

    static std::string deviceToString(Autonav::Device device)
    {
        switch (device)
        {
        case Device::STEAM_TRANSLATOR:
            return "STEAM_TRANSLATOR";
        case Device::MANUAL_CONTROL:
            return "MANUAL_CONTROL";
        case Device::DISPLAY_NODE:
            return "DISPLAY";
        case Device::SERIAL_IMU:
            return "IMU_TRANSLATOR";
        case Device::SERIAL_CAN:
            return "CAN_TRANSLATOR";
        case Device::LOGGING:
            return "LOGGER";
        default:
            return "UNKNOWN";
        }
    }

    static std::string deviceStateToString(Autonav::State::DeviceState state)
    {
        switch (state)
        {
        case State::DeviceState::OFF:
            return "Off";
        case State::DeviceState::STANDBY:
            return "Standby";
        case State::DeviceState::READY:
            return "Ready";
        case State::DeviceState::OPERATING:
            return "Operating";
        default:
            return "Unknown";
        }
    }

    namespace CanBus
    {
        enum MessageID
        {
            ESTOP = 0,
            MOBILITY_STOP = 1,
            RESET_MOTOR_CONTROLLER = 8,
            MOBILITY_START = 9,
            WRITE_MOTORS = 10,
            WRITE_SAFETY_LIGHTS = 13,
            ODOMETRY_FEEDBACK = 14
        };
    }

    namespace Configuration
    {
        class Conbus
        {
        public:
            Conbus(Device device, rclcpp::Node* node);
            ~Conbus();

        public:
            void write(uint8_t registerAddress, std::vector<uint8_t> data);
            void write(uint8_t registerAddress, uint8_t data);
            void write(uint8_t registerAddress, uint16_t data);
            void write(uint8_t registerAddress, uint32_t data);
            void write(uint8_t registerAddress, uint64_t data);
            void write(uint8_t registerAddress, int8_t data);
            void write(uint8_t registerAddress, int16_t data);
            void write(uint8_t registerAddress, int32_t data);
            void write(uint8_t registerAddress, int64_t data);
            void write(uint8_t registerAddress, float data);
            void write(uint8_t registerAddress, bool data);

            void writeTo(Device device, uint8_t registerAddress, std::vector<uint8_t> data);
            void writeTo(Device device, uint8_t registerAddress, uint8_t data);
            void writeTo(Device device, uint8_t registerAddress, uint16_t data);
            void writeTo(Device device, uint8_t registerAddress, uint32_t data);
            void writeTo(Device device, uint8_t registerAddress, uint64_t data);
            void writeTo(Device device, uint8_t registerAddress, int8_t data);
            void writeTo(Device device, uint8_t registerAddress, int16_t data);
            void writeTo(Device device, uint8_t registerAddress, int32_t data);
            void writeTo(Device device, uint8_t registerAddress, int64_t data);
            void writeTo(Device device, uint8_t registerAddress, float data);
            void writeTo(Device device, uint8_t registerAddress, bool data);

            template <typename T>
            T read(uint8_t registerAddress);

            void requestRemoteRegister(Device device, uint8_t registerAddress);
            void requestAllRemoteRegisters(Device device);

        private:
            void publishWrite(Device device, uint8_t address, std::vector<uint8_t> data);
            void publishRead(Device device, uint8_t address);
            void onConbusInstruction(const autonav_msgs::msg::ConBusInstruction::SharedPtr msg);

        private:
            Device _device;
            std::map<uint8_t, std::map<uint8_t, std::vector<uint8_t>>> _registers;

            rclcpp::Publisher<autonav_msgs::msg::ConBusInstruction>::SharedPtr _conbusPublisher;
            rclcpp::Subscription<autonav_msgs::msg::ConBusInstruction>::SharedPtr _conbusSubscriber;
        };
    }

    namespace ROS
    {
        class AutoNode : public rclcpp::Node
        {
        public:
            AutoNode(Autonav::Device device, std::string node_name);
            ~AutoNode();

            void setSystemState(State::SystemState state);
            void setDeviceState(State::DeviceState state);

        protected:
            Autonav::Device _device;
            State::SystemState _systemState;
            State::DeviceState _deviceState;
            std::map<Autonav::Device, State::DeviceState> _deviceStates;

            Configuration::Conbus _config;

        protected:
            void onSystemState(const autonav_msgs::msg::SystemState::SharedPtr msg);
            void onDeviceState(const autonav_msgs::msg::DeviceState::SharedPtr msg);

        private:
            rclcpp::Publisher<autonav_msgs::msg::SystemState>::SharedPtr _systemStatePublisher;
            rclcpp::Publisher<autonav_msgs::msg::DeviceState>::SharedPtr _deviceStatePublisher;
            rclcpp::Subscription<autonav_msgs::msg::SystemState>::SharedPtr _systemStateSubscriber;
            rclcpp::Subscription<autonav_msgs::msg::DeviceState>::SharedPtr _deviceStateSubscriber;
        };
    }
}