#pragma once

#include "scr_msgs/msg/performance_result.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/publisher.hpp"
#include <stdint.h>
#include <string.h>
#include <map>

namespace SCR
{
    class Performance
    {
    public:
        Performance();
        Performance(const std::string &node, rclcpp::Publisher<scr_msgs::msg::PerformanceResult>::SharedPtr performancePublisher);
        ~Performance();

        void start(const std::string &name);
        void end(const std::string &name);

    private:
        void publishData(const std::string &name);

    private:
        std::map<std::string, int64_t> timers;
        std::map<std::string, std::vector<int64_t>> history;
        int64_t minimum = 0;
        int64_t maximum = 0;
        std::string node;

        rclcpp::Publisher<scr_msgs::msg::PerformanceResult>::SharedPtr resultsPublisher;
    };
}