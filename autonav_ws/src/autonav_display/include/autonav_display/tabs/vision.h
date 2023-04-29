#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.hpp>
#include "opencv4/opencv2/opencv.hpp"

#include "scr_core/utils.h"
#include "scr_core/device_state.h"
#include "scr_core/node.h"
#include "imgui.h"

void ShowImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // Decompress using cv_bridge
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("autonav_display"), "cv_bridge exception: %s", e.what());
        return;
    }

    auto mat = cv_ptr->image;
    auto width = mat.cols;
    auto height = mat.rows;

    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, mat.data);
    glGenerateMipmap(GL_TEXTURE_2D);
        
    // Render
    ImGui::Image((void *)(intptr_t)texture, ImVec2(width, height), ImVec2(0, 0), ImVec2(1, 1));

    // Cleanup
    glDeleteTextures(1, &texture);
    cv_ptr.reset();
}

void ShowVision(SCR::Node *node)
{
    static rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rawImageSubscriber = nullptr;
    static sensor_msgs::msg::CompressedImage::SharedPtr rawImage =  nullptr;

    static rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr filteredImageSubscriber = nullptr;
    static sensor_msgs::msg::CompressedImage::SharedPtr filteredImage =  nullptr;

    static rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr depthImageSubscriber = nullptr;
    static sensor_msgs::msg::CompressedImage::SharedPtr depthImage =  nullptr;

    if (rawImageSubscriber == nullptr)
    {
        rawImageSubscriber = node->create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/camera/compressed", 20, [](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            rawImage = msg;
        });

        filteredImageSubscriber = node->create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/camera/filtered", 20, [](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            filteredImage = msg;
        });

        depthImageSubscriber = node->create_subscription<sensor_msgs::msg::CompressedImage>("/autonav/cfg_space/expanded/image", 20, [](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            depthImage = msg;
        });
    }

    if (rawImage != nullptr)
    {
        ShowImage(rawImage);
    }

    if (filteredImage != nullptr)
    {
        ShowImage(filteredImage);
    }

    if (depthImage != nullptr)
    {
        ShowImage(depthImage);
    }
}