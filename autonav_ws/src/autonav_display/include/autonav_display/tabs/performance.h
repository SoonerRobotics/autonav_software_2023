#include "scr_msgs/msg/performance_result.hpp"
#include "scr_core/node.h"
#include "implot.h"
#include "imgui.h"

void ShowPerformance(SCR::Node *node)
{
    static std::map<std::string, std::map<std::string, std::vector<int32_t>>> performanceHistory;
    static std::map<std::string, std::map<std::string, scr_msgs::msg::PerformanceResult>> latestResults;
    static rclcpp::Subscription<scr_msgs::msg::PerformanceResult>::SharedPtr performanceSubscription = nullptr;

    if (performanceSubscription == nullptr)
    {
        performanceSubscription = node->create_subscription<scr_msgs::msg::PerformanceResult>(
            "/scr/performance", 20, [&](const scr_msgs::msg::PerformanceResult::SharedPtr msg) {
                latestResults[msg->node][msg->key] = *msg;
                performanceHistory[msg->node][msg->key].push_back(msg->latest);

                if (performanceHistory[msg->node][msg->key].size() > 250)
                {
                    performanceHistory[msg->node][msg->key].erase(performanceHistory[msg->node][msg->key].begin());
                }
            });
    }

    for (auto &entry : latestResults)
    {
        if (ImGui::CollapsingHeader(entry.first.c_str()))
        {
            for (auto &result : entry.second)
            {
                if (ImGui::TreeNode(result.first.c_str()))
                {
                    auto history = performanceHistory[entry.first][result.first];
                    auto results = result.second;

                    ImGui::Text("Average: %d ms", results.average);
                    ImGui::Text("Min: %d ms", results.minimum);
                    ImGui::Text("Max: %d ms ", results.maximum);
                    ImGui::Text("Latest: %d ms", results.latest);
                    ImGui::Text("Latest: %d ns", results.latest_ns);

                    if (ImPlot::BeginPlot("##PerformancePlot", ImVec2(750, 400)))
                    {
                        ImPlot::SetupAxes("Time [cycles]", "Time [ms]", ImPlotAxisFlags_AutoFit, 0);
                        ImPlot::SetupLegend(ImPlotLocation_NorthWest);

                        ImPlot::PlotLine("Duration", history.data(), history.size());
                        ImPlot::EndPlot();
                    }
                    ImGui::TreePop();
                    ImGui::Spacing();
                }
            }
        }
    }
}