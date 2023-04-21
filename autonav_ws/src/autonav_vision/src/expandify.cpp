#include "scr_core/node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.hpp>
#include "opencv4/opencv2/opencv.hpp"

namespace ExpandifyConstants
{
	const float VERTICAL_FOV = 2.75;
	const float HORIZONTAL_FOV = 3;
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
		maxRange = (int)(maxRange / (ExpandifyConstants::HORIZONTAL_FOV / 80.0));
		noGoRange = (int)(tempRange / (ExpandifyConstants::HORIZONTAL_FOV / 80.0));

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

		mapSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/raw", 10, std::bind(&Expandify::onConfigSpaceReceived, this, std::placeholders::_1));
		mapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/autonav/cfg_space/expanded", 10);
		imagePublisher = this->create_publisher<sensor_msgs::msg::CompressedImage>("/autonav/cfg_space/expanded/image", 10);

		setDeviceState(SCR::DeviceState::OPERATING);
	}

	void onConfigSpaceReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr cfg)
	{
		// Create a 80 * 80 1d int array
		std::vector<int8_t> data = std::vector<int8_t>(80 * 80);
		std::fill(data.begin(), data.end(), 0);

		for (int x = 0; x < 80; x++)
		{
			for (int y = 1; y < 80; y++)
			{
				if(cfg->data.at(x + y * 80) > 0)
				{
					for (Circle& circle : circles)
					{
						auto idx = (x + circle.x) + 80 * (y + circle.y);
						auto expr_x = (x + circle.x) < 80 && (x + circle.x) >= 0;
						auto expr_y = (y + circle.y) < 80 && (y + circle.y) >= 0;
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

		// Remap data from 0 to 255
		for (int i = 0; i < data.size(); i++)
		{
			if (data.at(i) == 0)
			{
				data.at(i) = 0;
			} else if (data.at(i) == 100)
			{
				data.at(i) = 255;
			} else
			{
				data.at(i) = 255 - data.at(i);
			}
		}

		// Generate cv image and publish compreseed image
		cv::Mat image = cv::Mat(80, 80, CV_8UC1, data.data());
		cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);

		cv::circle(image, cv::Point(40, 78), 2, cv::Scalar(255, 0, 0), 2);

		// Increase image isze
		cv::resize(image, image, cv::Size(800, 800), 0, 0, cv::INTER_NEAREST);

		auto cmpr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toCompressedImageMsg();
		cmpr->header.stamp = this->now();
		cmpr->header.frame_id = "autonav_vision_expandifier";
		cmpr->format = "jpeg";
		imagePublisher->publish(*cmpr);
	}

private:
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscriber;
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;
	rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr imagePublisher;

	nav_msgs::msg::MapMetaData map;
	
	float maxRange = 0.5;
	float noGoPercent = 0.5;
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