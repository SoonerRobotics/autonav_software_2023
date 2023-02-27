#include "autonav_msgs/msg/con_bus_instruction.hpp"
#include "autonav_msgs/srv/set_system_state.hpp"
#include "autonav_msgs/srv/set_device_state.hpp"
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
        MANUAL_CONTROL_STEAM = 101,
        MANUAL_CONTROL_XBOX = 102,
        DISPLAY_NODE = 103,
        SERIAL_IMU = 104,
        SERIAL_CAN = 105,
        LOGGING = 106
    };

    namespace State
    {
        enum DeviceState
        {
            OFF = 0,
            STANDBY = 1,
            READY = 2,
            OPERATING = 3
        };

        enum SystemState
        {
            DISABLED = 0,
            AUTONOMOUS = 1,
            MANUAL = 2
        };
    }

    static const char* deviceToString(Autonav::Device device)
    {
        switch (device)
        {
        case Device::STEAM_TRANSLATOR:
            return "STEAM_TRANSLATOR";
        case Device::MANUAL_CONTROL_STEAM:
            return "MANUAL_CONTROL_STEAM";
        case Device::MANUAL_CONTROL_XBOX:
            return "MANUAL_CONTROL_XBOX";
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

    static const char* deviceStateToString(Autonav::State::DeviceState state)
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
		const float FLOAT_PRECISION = 10000000.0f;
		const int MAX_DEVICE_ID = 200;

        enum ConbusOpcode
        {
            READ = 0,
            READ_ACK = 1,
            WRITE = 2,
            WRITE_ACK = 3,
            READ_ALL = 4
        };

        class Conbus
        {
        public:
            Conbus(Device device, rclcpp::Node* node);
            ~Conbus();

        public:
            void writeBytes(uint8_t registerAddress, std::vector<uint8_t> data);
            void write(uint8_t registerAddress, int32_t data);
            void write(uint8_t registerAddress, float data);
            void write(uint8_t registerAddress, bool data);

            void writeTo(Device device, uint8_t registerAddress, int32_t data);
            void writeTo(Device device, uint8_t registerAddress, float data);
            void writeTo(Device device, uint8_t registerAddress, bool data);

            template <typename T>
            T read(uint8_t registerAddress);

            template <typename T>
            T read(Device device, uint8_t registerAddress);

            void requestRemoteRegister(Device device, uint8_t registerAddress);
            void requestAllRemoteRegistersFrom(Device device);
            void requestAllRemoteRegisters();

            // Create iterators to read through devices
            std::map<uint8_t, std::map<uint8_t, std::vector<uint8_t>>>::iterator begin();
            std::map<uint8_t, std::map<uint8_t, std::vector<uint8_t>>>::iterator end();

            std::map<uint8_t, std::vector<uint8_t>> getRegistersForDevice(Device device);
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

            bool setSystemState(State::SystemState state);
            bool setDeviceState(State::DeviceState state);

            Configuration::Conbus _config;
            Autonav::Device _device;
            State::SystemState _systemState;
            State::DeviceState _deviceState;

            virtual void setup() {}
            virtual void operate() {}

        protected:
            virtual void onSystemState(const autonav_msgs::msg::SystemState::SharedPtr msg);
            virtual void onDeviceState(const autonav_msgs::msg::DeviceState::SharedPtr msg);

        private:
            rclcpp::Subscription<autonav_msgs::msg::SystemState>::SharedPtr _systemStateSubscriber;
            rclcpp::Subscription<autonav_msgs::msg::DeviceState>::SharedPtr _deviceStateSubscriber;

            rclcpp::Client<autonav_msgs::srv::SetDeviceState>::SharedPtr _deviceStateClient;
            rclcpp::Client<autonav_msgs::srv::SetSystemState>::SharedPtr _systemStateClient;
        };
    }
}