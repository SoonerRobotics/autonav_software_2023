#include "scr_msgs/msg/performance_result.hpp"
#include "scr_core/performance.h"
#include "rclcpp/rclcpp.hpp"

namespace SCR
{
    Performance::Performance()
    {
    }

    Performance::Performance(const std::string &node, rclcpp::Publisher<scr_msgs::msg::PerformanceResult>::SharedPtr performancePublisher)
    {
        resultsPublisher = performancePublisher;
        this->node = node;
    }

    Performance::~Performance()
    {
    }

    void Performance::start(const std::string &name)
    {
        timers[name] = rclcpp::Clock().now().nanoseconds();
    }

    void Performance::end(const std::string &name)
    {
        auto now = rclcpp::Clock().now().nanoseconds();
        auto duration = now - timers[name];
        history[name].push_back(duration);

        if (history[name].size() > 500)
        {
            history[name].erase(history[name].begin());
        }

        if (minimum == 0 || duration < minimum)
        {
            minimum = duration;
        }

        if (maximum == 0 || duration > maximum)
        {
            maximum = duration;
        }

        publishData(name);
    }

    void Performance::publishData(const std::string &name)
    {
        auto results = scr_msgs::msg::PerformanceResult();
        results.key = name;
        results.average = 0;
        results.minimum = minimum / 1000000;
        results.maximum = maximum / 1000000;
        results.latest = history[name].back() / 1000000;
        results.latest_ns = history[name].back();
        results.node = node;

        auto size = history[name].size();
        for (auto &value : history[name])
        {
            results.average += value / 1000000;
        }
        results.average /= size;
        resultsPublisher->publish(results);
    }
}