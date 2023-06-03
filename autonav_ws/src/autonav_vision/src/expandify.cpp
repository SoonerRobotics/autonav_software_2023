#include "scr_core/node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

namespace ExpandifyConstants
{
	const float VERTICAL_FOV = 2.75;
	const float HORIZONTAL_FOV = 3;
	const float MAP_RES_F = 80.0f;
	const int MAP_RES = 80;
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
				if (maxRange * noGoPercent <= sqrt(x * x + y * y) && sqrt(x * x + y * y) < maxRange)
				{
					circles.push_back(Circle{x, y, sqrt(x * x + y * y)});
				}
			}
		}

		rawMapSubscriber = create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/raw", 20, std::bind(&Expandify::onConfigSpaceReceived, this, std::placeholders::_1));
		expandedMapPublisher = create_publisher<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 20);
		debugPublisher = create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/cfg_space/expanded/image", 20);

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

		performance.start("Expandification");

		std::vector<int8_t> cfg_space = std::vector<int8_t>(ExpandifyConstants::MAP_RES * ExpandifyConstants::MAP_RES);
		std::fill(cfg_space.begin(), cfg_space.end(), 0);

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
							auto val = cfg_space.at(idx);
							auto linear = 100 - ((circle.radius - noGoRange) / (maxRange - noGoRange) * 100);

							if (circle.radius <= noGoRange)
							{
								cfg_space.at(idx) = 100;
							} else if (cfg_space.at(idx) <= 100 && val <= linear)
							{
								cfg_space.at(idx) = int(linear);
							}
						}
					}
				}
			}
		}

		auto newSpace = nav_msgs::msg::OccupancyGrid();
		newSpace.info = map;
		newSpace.data = cfg_space;
		expandedMapPublisher->publish(newSpace);

		cv::Mat image = cv::Mat(ExpandifyConstants::MAP_RES, ExpandifyConstants::MAP_RES, CV_8UC1, cfg_space.data());
		cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
		cv::resize(image, image, cv::Size(800, 800), 0, 0, cv::INTER_NEAREST);

		auto compressed = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toCompressedImageMsg();
		compressed->header.stamp = this->now();
		compressed->header.frame_id = "map";
		compressed->format = "jpeg";
		debugPublisher->publish(*compressed);

		performance.end("Expandification");
	}

private:
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr rawMapSubscriber;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr expandedMapPublisher;
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debugPublisher;

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