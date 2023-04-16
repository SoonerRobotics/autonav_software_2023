#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.hpp>
#include "opencv4/opencv2/opencv.hpp"

#include "autonav_libs/utils.h"
#include "autonav_libs/device_state.h"
#include "autonav_libs/node.h"
#include "imgui.h"

void BindCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr msg, GLuint *image_id, int *width, int *height)
{
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
    if (*image_id == 0)
    {
        glGenTextures(1, image_id);
    }

    *width = mat.cols;
    *height = mat.rows;

    glBindTexture(GL_TEXTURE_2D, *image_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mat.cols, mat.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, mat.data);
    glGenerateMipmap(GL_TEXTURE_2D);
}

void ShowVision(Autonav::AutoNode *node)
{
    static rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rawImageSubscriber = nullptr;
    static sensor_msgs::msg::CompressedImage rawImage;
    static GLuint rawImageId = -1;
    static int rawImageHeight = 0;
    static int rawImageWidth = 0;

    if (rawImageSubscriber == nullptr)
    {
        rawImageSubscriber = node->create_subscription<sensor_msgs::msg::CompressedImage>(
            "igvc/camera/compressed", 20, [&](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                rawImage = *msg;
                BindCompressedImage(msg, &rawImageId, &rawImageWidth, &rawImageHeight);
            }
        );
    }

    if (rawImageId != 0)
    {
        ImGui::Image((void *)(intptr_t)rawImageId, ImVec2(rawImageWidth, rawImageHeight), ImVec2(0, 0), ImVec2(1, 1));
    }
}