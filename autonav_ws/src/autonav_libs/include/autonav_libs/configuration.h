#pragma once

#include "autonav_msgs/msg/configuration_instruction.hpp"
#include "rclcpp/rclcpp.hpp"
#include <stdint.h>
#include <string.h>
#include <map>

namespace Autonav
{
    class Configuration
    {
        public:
            Configuration();
            Configuration(int64_t id, rclcpp::Subscription<autonav_msgs::msg::ConfigurationInstruction>::SharedPtr configSubscriber, rclcpp::Publisher<autonav_msgs::msg::ConfigurationInstruction>::SharedPtr configPublisher);
            ~Configuration();

            template <typename T>
            T get(uint8_t address);

            template <typename T>
            T get(uint8_t address, T default_value);

            template <typename T>
            T get(int64_t device, uint8_t address);

            template <typename T>
            T get(int64_t device, uint8_t address, T default_value);

            template <typename T>
            void set(uint8_t address, T value);

            template <typename T>
            void set(int64_t device, uint8_t address, T value);

            void recache();
			void onConfigurationInstruction(const autonav_msgs::msg::ConfigurationInstruction::SharedPtr msg);

        private:
            int64_t id;
            std::map<int64_t, std::map<uint8_t, std::vector<uint8_t>>> cache;
			rclcpp::Subscription<autonav_msgs::msg::ConfigurationInstruction>::SharedPtr configSubscriber;
			rclcpp::Publisher<autonav_msgs::msg::ConfigurationInstruction>::SharedPtr configPublisher;

        private:
            enum Opcode
            {
                GET = 0,
                SET = 1,
                GET_ACK = 2,
                SET_ACK = 3,
                GET_ALL = 4
            };
    };
}