#include "scr_core/node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace ExpandifyConstants
{
	const float VERTICAL_FOV = 2.75;
	const float HORIZONTAL_FOV = 3;
}

struct Circle
{
	int x;
	int y;
	float radius;
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

		noGoRange = (int)(maxRange * noGoPercent);
		maxRange = (int)(maxRange / (ExpandifyConstants::HORIZONTAL_FOV / 80));
		noGoRange = (int)(noGoRange / (ExpandifyConstants::HORIZONTAL_FOV / 80));

		std::vector<int> range = {};
		for (int i = -maxRange; i <= maxRange; i++)
		{
			range.push_back(i);
		}
		circles.push_back(Circle{0, 0, 0});

		for (int x : range)
		{
			for (int y : range)
			{
				if (noGoRange <= sqrt(x * x + y * y) && sqrt(x * x + y * y) < maxRange)
				{
					circles.push_back(Circle{x, y, (float)sqrt(x * x + y * y)});
				}
			}
		}

		mapSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/raw", 10, std::bind(&Expandify::onConfigSpaceReceived, this, std::placeholders::_1));
		mapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 10);

		setDeviceState(SCR::DeviceState::OPERATING);
	}

	void onConfigSpaceReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr cfg)
	{
		// Create a 80 * 80 1d int array
		std::vector<int8_t> data = std::vector<int8_t>(80 * 80);
		for (int x = 0; x < 80; x++)
		{
			for (int y = 0; y < 80; y++)
			{
				if(cfg->data.at(x + y * 80) > 0)
				{
					for (Circle& circle : circles)
					{
						auto idx = (x + circle.x) + 80 * (y + circle.y);
						auto expr_x = (x + circle.x) < 80 && 0 <= (x + circle.x);
						auto expr_y = (y + circle.y) < 80 && 0 <= (y + circle.y);
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
		mapPublisher->publish(newSpace);
	}

private:
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscriber;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;

	nav_msgs::msg::MapMetaData map;
	
	float maxRange = 0.65;
	float noGoPercent = 0.70;
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