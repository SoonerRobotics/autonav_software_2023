#include "autonav_msgs/msg/con_bus_instruction.hpp"
#include "autonav_msgs/msg/device_state.hpp"
#include "autonav_msgs/msg/system_state.hpp"
#include "rclcpp/rclcpp.hpp"

#include <stdint.h>
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
            std::vector<State::DeviceState> _deviceStates;

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