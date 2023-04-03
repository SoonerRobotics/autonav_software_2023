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
            MANUAL = 2,
            SHUTDOWN = 3
        };
    }

    int32_t hash(std::string str)
    {
        int32_t asd = 5381;
        int c;

        for (int i = 0; i < str.length(); i++)
        {
            c = str[i];
            asd = ((asd << 5) + asd) + c; /* hash * 33 + c */
        }

        return asd & 0xFFFFFFFF;
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
            Conbus(std::string device, rclcpp::Node* node);
            ~Conbus();

        public:
            /**
             * @brief Write a vector of bytes to the local configuration
            */
            void writeBytes(uint8_t registerAddress, std::vector<uint8_t> data);
            /**
             * @brief Write a 32-bit integer to the local configuration
            */
            void write(uint8_t registerAddress, int32_t data);
            /**
             * @brief Write a 32-bit float to the local configuration
            */
            void write(uint8_t registerAddress, float data);
            /**
             * @brief Write a boolean to the local configuration
            */
            void write(uint8_t registerAddress, bool data);

            /**
             * @brief Write a 32-bit integer to a remote configuration
            */
            void writeTo(std::string device, uint8_t registerAddress, int32_t data);
            /**
             * @brief Write a 32-bit float to a remote configuration
            */
            void writeTo(std::string device, uint8_t registerAddress, float data);
            /**
             * @brief Write a boolean to a remote configuration
            */
            void writeTo(std::string device, uint8_t registerAddress, bool data);
            /**
             * @brief Write a vector of bytes to a remote configuration
            */
            void writeTo(std::string device, uint8_t registerAddress, std::vector<uint8_t> data);

            /**
             * @brief Read from the local configuration
            */
            template <typename T>
            T read(uint8_t registerAddress);

            /**
             * @brief Read from a remote configuration
            */
            template <typename T>
            T read(std::string device, uint8_t registerAddress);

            /**
             * @brief Gets a iterator to the beginning of the local configuration
            */
            std::map<uint8_t, std::map<uint8_t, std::vector<uint8_t>>>::iterator begin();
            /**
             * @brief Gets a iterator to the end of the local configuration
            */
            std::map<uint8_t, std::map<uint8_t, std::vector<uint8_t>>>::iterator end();

            /**
             * @brief Returns a map of all the registers for a device
            */
            std::map<uint8_t, std::vector<uint8_t>> getRegistersForDevice(std::string device);
        private:
            void publishWrite(std::string device, uint8_t address, std::vector<uint8_t> data);
            void publishRead(std::string device, uint8_t address);
            void onConbusInstruction(const autonav_msgs::msg::ConBusInstruction::SharedPtr msg);

        private:
            int32_t m_id;
            std::map<uint8_t, std::map<uint8_t, std::vector<uint8_t>>> m_registers;
            rclcpp::Publisher<autonav_msgs::msg::ConBusInstruction>::SharedPtr m_conbusPublisher;
            rclcpp::Subscription<autonav_msgs::msg::ConBusInstruction>::SharedPtr m_conbusSubscriber;
        };
    }

    namespace ROS
    {
        class AutoNode : public rclcpp::Node
        {
        public:
            AutoNode(std::string node_name);
            ~AutoNode();

            bool setSystemState(State::SystemState state);
            bool setDeviceState(State::DeviceState state);

            Configuration::Conbus config;

            /**
             * @brief Called when the node is initialized
            */
            virtual void setup() {}
            /**
             * @brief Called when the node is switched into the operating state
            */
            virtual void operate() {}
            /**
             * @brief Called when the node is switched out of the operating state
            */
            virtual void deoperate() {}

            State::SystemState getSystemState();
            State::DeviceState getDeviceState();
            State::DeviceState getDeviceState(std::string device);

        protected:
            virtual void onSystemState(const autonav_msgs::msg::SystemState::SharedPtr msg);
            virtual void onDeviceState(const autonav_msgs::msg::DeviceState::SharedPtr msg);
            /**
             * @brief Terminates the local node by killing the process
             * @note This does NOT tell the state manager that its terminating
            */
            void terminate();
            bool m_isSimulator;
            int32_t m_id;

        private:
            void onInitializeTimer();

        private:
            rclcpp::Subscription<autonav_msgs::msg::SystemState>::SharedPtr m_systemStateSubscriber;
            rclcpp::Subscription<autonav_msgs::msg::DeviceState>::SharedPtr m_deviceStateSubscriber;

            rclcpp::Client<autonav_msgs::srv::SetDeviceState>::SharedPtr m_deviceStateClient;
            rclcpp::Client<autonav_msgs::srv::SetSystemState>::SharedPtr m_systemStateClient;

            State::SystemState m_systemState;
            std::map<int32_t, State::DeviceState> m_deviceStates;

            rclcpp::TimerBase::SharedPtr _initializeTimer;
        };
    }
}