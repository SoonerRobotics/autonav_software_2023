#include "autonav_msgs/msg/con_bus_instruction.hpp"
#include "rclcpp/rclcpp.hpp"

#include <stdint.h>
#include <string.h>

namespace Autonav
{
    namespace ConBus
    {
        struct register_entry
        {
            uint8_t address;
            uint8_t length;
            uint8_t *data;
        };

        enum Opcode
        {
            READ = 0,
            READ_ACK = 1,
            WRITE = 2,
            WRITE_ACK = 3,
            READ_ALL = 4
        };

        enum Device
        {
            MOTOR_CONTROLLER = 0,
            ESTOP_RECEIVER = 1,
            SAFETY_LIGHTS = 2
        };

        class Controller
        {
        private:
            std::vector<register_entry> _registers;
            int _device;
            rclcpp::Publisher<autonav_msgs::msg::ConBusInstruction>::SharedPtr _publisher;

        public:
            Controller();
            Controller(int device, rclcpp::Publisher<autonav_msgs::msg::ConBusInstruction>::SharedPtr publisher);
            ~Controller();

            void read(uint8_t device, uint8_t address, register_entry* entry);
            void write(uint8_t device, uint8_t address, uint8_t *data, uint8_t length);
            void onInstruction(const autonav_msgs::msg::ConBusInstruction::SharedPtr msg);
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
}