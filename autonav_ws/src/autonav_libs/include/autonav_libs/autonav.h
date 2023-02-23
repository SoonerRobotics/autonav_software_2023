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

            void read(uint8_t device, uint8_t address, register_entry *entry);
            void write(uint8_t device, uint8_t address, uint8_t *data, uint8_t length);
            void onInstruction(const autonav_msgs::msg::ConBusInstruction::SharedPtr msg);
            void setDefaultRegisters(std::vector<register_entry> defaultRegisters);

            int32_t asInt32(uint8_t *data)
            {
                return (int32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
            }

            uint32_t asUInt32(uint8_t *data)
            {
                return (uint32_t)((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]);
            }

            int16_t asInt16(uint8_t *data)
            {
                return (int16_t)((data[0] << 8) | data[1]);
            }

            uint16_t asUInt16(uint8_t *data)
            {
                return (uint16_t)((data[0] << 8) | data[1]);
            }

            int8_t asInt8(uint8_t *data)
            {
                return (int8_t)data[0];
            }

            uint8_t asUInt8(uint8_t *data)
            {
                return (uint8_t)data[0];
            }

            float asFloat(uint8_t *data)
            {
                return (float)asInt32(data) / 1000000.0;
            }

            long asLong(uint8_t *data)
            {
                return (long)asInt32(data);
            }

            unsigned long asULong(uint8_t *data)
            {
                return (unsigned long)asUInt32(data);
            }

            // Address Reads

            long readLong(uint8_t address)
            {
                return asLong(_registers[address].data);
            }

            unsigned long readUnsignedLong(uint8_t address)
            {
                return asULong(_registers[address].data);
            }

            int32_t readInt32(uint8_t address)
            {
                return asInt32(_registers[address].data);
            }

            uint32_t readUInt32(uint8_t address)
            {
                return asUInt32(_registers[address].data);
            }

            int16_t readInt16(uint8_t address)
            {
                return asInt16(_registers[address].data);
            }

            uint16_t readUInt16(uint8_t address)
            {
                return asUInt16(_registers[address].data);
            }

            int8_t readInt8(uint8_t address)
            {
                return asInt8(_registers[address].data);
            }

            uint8_t readUInt8(uint8_t address)
            {
                return asUInt8(_registers[address].data);
            }

            float readFloat(uint8_t address)
            {
                return asFloat(_registers[address].data);
            }

            // Write Address


            void writeLong(uint8_t address, long value)
            {
                uint8_t *data = (uint8_t *)&value;
                write(_device, address, data, 4);
            }

            void writeUnsignedLong(uint8_t address, unsigned long value)
            {
                uint8_t *data = (uint8_t *)&value;
                write(_device, address, data, 4);
            }

            void writeInt32(uint8_t address, int32_t value)
            {
                uint8_t *data = (uint8_t *)&value;
                write(_device, address, data, 4);
            }

            void writeUInt32(uint8_t address, uint32_t value)
            {
                uint8_t *data = (uint8_t *)&value;
                write(_device, address, data, 4);
            }

            void writeInt16(uint8_t address, int16_t value)
            {
                uint8_t *data = (uint8_t *)&value;
                write(_device, address, data, 2);
            }

            void writeUInt16(uint8_t address, uint16_t value)
            {
                uint8_t *data = (uint8_t *)&value;
                write(_device, address, data, 2);
            }

            void writeInt8(uint8_t address, int8_t value)
            {
                uint8_t *data = (uint8_t *)&value;
                write(_device, address, data, 1);
            }

            void writeUInt8(uint8_t address, uint8_t value)
            {
                uint8_t *data = (uint8_t *)&value;
                write(_device, address, data, 1);
            }

            void writeFloat(uint8_t address, float value)
            {
                int32_t data = (int32_t)(value * 1000000.0);
                write(_device, address, (uint8_t *)&data, 4);
            }

            // bruh

            register_entry &operator[](int index)
            {
                return _registers[index];
            }
        };

        class ConbusNode : public rclcpp::Node
        {
        public:
            ConbusNode(int device, std::string node_name);
            ~ConbusNode();

            Autonav::ConBus::Controller getController();

        protected:
            Autonav::ConBus::Controller _controller;

        private:
            rclcpp::Publisher<autonav_msgs::msg::ConBusInstruction>::SharedPtr _publisher;
            rclcpp::Subscription<autonav_msgs::msg::ConBusInstruction>::SharedPtr _subscriber;
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