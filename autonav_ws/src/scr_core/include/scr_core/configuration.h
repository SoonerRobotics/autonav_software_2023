#pragma once

#include "scr_msgs/msg/configuration_instruction.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/publisher.hpp"
#include <stdint.h>
#include <string.h>
#include <map>

namespace SCR
{
    class Configuration
    {
        public:
            Configuration();
            Configuration(
                int64_t id,
                rclcpp::Subscription<scr_msgs::msg::ConfigurationInstruction>::SharedPtr configSubscriber,
                rclcpp::Publisher<scr_msgs::msg::ConfigurationInstruction>::SharedPtr configPublisher,
                rclcpp::Subscription<std_msgs::msg::String>::SharedPtr loadSubscription,
                rclcpp::Publisher<std_msgs::msg::String>::SharedPtr loadPublisher
            );
            ~Configuration();

            template <typename T>
            T get(uint8_t address);

            template <typename T>
            T get(int64_t device, uint8_t address);

            template <typename T>
            void set(uint8_t address, T value);

            template <typename T>
            void set(int64_t device, uint8_t address, T value);

            bool has(int64_t device, uint8_t address);

            void recache();
			void onConfigurationInstruction(const scr_msgs::msg::ConfigurationInstruction::SharedPtr msg);

            bool hasDevice(int64_t device);

            std::map<int64_t, std::map<uint8_t, std::vector<uint8_t>>> getCache();

            std::vector<std::string> getPresets();
            void load(const std::string& preset);
            void save(const std::string& preset);
            void onPresetChanged(std_msgs::msg::String::SharedPtr msg);
            bool hasLoadedPreset();
            std::string getActivePreset();
            void loadLocalPresets();
            void deleteActivePreset();
            void deleteAllPresets();

        private:
            int64_t id;
            std::map<int64_t, std::map<uint8_t, std::vector<uint8_t>>> cache = {};
			rclcpp::Subscription<scr_msgs::msg::ConfigurationInstruction>::SharedPtr configSubscriber;
			rclcpp::Publisher<scr_msgs::msg::ConfigurationInstruction>::SharedPtr configPublisher;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr loadSubscription;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr loadPublisher;
            std::vector<std::string> presets;
            std::string preset;
            bool loading = false;

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