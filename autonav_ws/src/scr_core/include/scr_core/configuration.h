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
                std::string id,
                rclcpp::Subscription<scr_msgs::msg::ConfigurationInstruction>::SharedPtr configSubscriber,
                rclcpp::Publisher<scr_msgs::msg::ConfigurationInstruction>::SharedPtr configPublisher,
                rclcpp::Subscription<std_msgs::msg::String>::SharedPtr loadSubscription,
                rclcpp::Publisher<std_msgs::msg::String>::SharedPtr loadPublisher
            );
            ~Configuration();

            template <typename T>
            T get(std::string address);

            template <typename T>
            T get(std::string device, std::string address);

            template <typename T>
            void set(std::string address, T value);

            template <typename T>
            void set(std::string device, std::string address, T value);

            bool has(std::string device, std::string address);

            void recache();
			void onConfigurationInstruction(const scr_msgs::msg::ConfigurationInstruction::SharedPtr msg);

            bool hasDevice(std::string device);

            std::map<std::string, std::map<std::string, std::vector<uint8_t>>> getCache();

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
            std::string id;
            std::map<std::string, std::map<std::string, std::vector<uint8_t>>> cache = {};
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