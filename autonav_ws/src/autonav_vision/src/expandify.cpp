#include "scr_core/node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace ExpandifyConstants
{
	const float VERTICAL_FOV = 2.75;
	const float HORIZONTAL_FOV = 3;
	const float MAP_RES_F = 100.0f;
	const int MAP_RES = 100;
}

struct Circle
{
	int x;
	int y;
	double radius;
};

class Expandify : public SCR::Node
{
public:
	Expandify() : SCR::Node("autonav_vision_expandifier") {}
	~Expandify() {}

	void configure() override
	{
		map = nav_msgs::msg::MapMetaData();
		map.width = 100;
		map.height = 100;
		map.resolution = 0.1;
		map.origin = geometry_msgs::msg::Pose();
		map.origin.position.x = -10.0;
		map.origin.position.y = -10.0;

		auto tempRange = maxRange * noGoPercent;
		maxRange = (int)(maxRange / (ExpandifyConstants::HORIZONTAL_FOV / ExpandifyConstants::MAP_RES_F));
		noGoRange = (int)(tempRange / (ExpandifyConstants::HORIZONTAL_FOV / ExpandifyConstants::MAP_RES_F));

		circles.push_back(Circle{0, 0, 0});
		for (int x = -maxRange; x <= maxRange; x++)
		{
			for (int y = -maxRange; y <= maxRange; y++)
			{
				if (maxRange * noGoPercent <= sqrt(x * x + y * y) && sqrt(x * x + y * y) < maxRange && (x + y) % 3 == 0)
				{
					circles.push_back(Circle{x, y, sqrt(x * x + y * y)});
				}
			}
		}

		rawMapSubscriber = create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/raw", 10, std::bind(&Expandify::onConfigSpaceReceived, this, std::placeholders::_1));
		expandedMapPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 10);

		setDeviceState(SCR::DeviceState::OPERATING);
	}

	void transition(scr_msgs::msg::SystemState old, scr_msgs::msg::SystemState updated) override
	{
		if (updated.state == SCR::SystemState::AUTONOMOUS && getDeviceState() == SCR::DeviceState::READY)
		{
			setDeviceState(SCR::DeviceState::OPERATING);
		}

		if (updated.state != SCR::SystemState::AUTONOMOUS && getDeviceState() == SCR::DeviceState::OPERATING)
		{
			setDeviceState(SCR::DeviceState::READY);
		}
	}

	void onConfigSpaceReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr cfg)
	{
		if (getDeviceState() != SCR::DeviceState::OPERATING || getSystemState().state != SCR::SystemState::AUTONOMOUS)
		{
			return;
		}

		std::vector<int8_t> data = std::vector<int8_t>(ExpandifyConstants::MAP_RES * ExpandifyConstants::MAP_RES);
		std::fill(data.begin(), data.end(), 0);

		for (int x = 0; x < ExpandifyConstants::MAP_RES; x++)
		{
			for (int y = 1; y < ExpandifyConstants::MAP_RES; y++)
			{
				if(cfg->data.at(x + y * ExpandifyConstants::MAP_RES) > 0)
				{
					for (Circle& circle : circles)
					{
						auto idx = (x + circle.x) + ExpandifyConstants::MAP_RES * (y + circle.y);
						auto expr_x = (x + circle.x) < ExpandifyConstants::MAP_RES && (x + circle.x) >= 0;
						auto expr_y = (y + circle.y) < ExpandifyConstants::MAP_RES && (y + circle.y) >= 0;
						if (expr_x && expr_y)
						{
							auto val = cfg->data.at(idx);
							auto linear = 100 - ((circle.radius - noGoRange) / (maxRange - noGoRange) * 100);

							if (circle.radius <= noGoRange)
							{
								data.at(idx) = 100;
							} else if (circle.radius <= 100 && val <= linear)
							{
								data.at(idx) = int(linear);
							}
						}
					}
				}
			}
		}

		auto newSpace = nav_msgs::msg::OccupancyGrid();
		newSpace.info = map;
		newSpace.data = data;
		expandedMapPublisher->publish(newSpace);
	}

private:
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr rawMapSubscriber;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr expandedMapPublisher;

	nav_msgs::msg::MapMetaData map;
	
	float maxRange = 0.55;
	float noGoPercent = 0.75;
	int noGoRange = 0;
	std::vector<Circle> circles;
};

int main(int, char **)
{
	rclcpp::init(0, NULL);
	auto node = std::make_shared<Expandify>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}