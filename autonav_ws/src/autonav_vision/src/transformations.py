#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np

from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

from scr_msgs.msg import SystemState
from scr_core.node import Node
from scr_core.state import DeviceStateEnum

g_bridge = CvBridge()

g_mapData = MapMetaData()
g_mapData.width = 200
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0

LOWER_HUE = 0
LOWER_SATURATION = 1
LOWER_VALUE = 2
UPPER_HUE = 3
UPPER_SATURATION = 4
UPPER_VALUE = 5
BLUR = 6
BLUR_ITERATIONS = 7
REGION_OF_DISINTEREST_TL = 8
REGION_OF_DISINTEREST_TR = 9


class ImageTransformer(Node):
    def __init__(self):
        super().__init__("autonav_vision_transformer")

    def configure(self):
        self.config.setInt(LOWER_HUE, 0)
        self.config.setInt(LOWER_SATURATION, 0)
        self.config.setInt(LOWER_VALUE, 35)
        self.config.setInt(UPPER_HUE, 255)
        self.config.setInt(UPPER_SATURATION, 100)
        self.config.setInt(UPPER_VALUE, 150)
        self.config.setInt(BLUR, 5)
        self.config.setInt(BLUR_ITERATIONS, 3)
        self.config.setInt(REGION_OF_DISINTEREST_TL, 50)
        self.config.setInt(REGION_OF_DISINTEREST_TR, 50)

        self.m_cameraSubscriber = self.create_subscription(CompressedImage, "/autonav/camera/compressed", self.onImageReceived, 1)
        self.m_laneMapPublisher = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/raw", 1)
        self.m_lanePreviewPublisher = self.create_publisher(CompressedImage, "/autonav/camera/filtered", 1)

        self.setDeviceState(DeviceStateEnum.OPERATING)

    def transition(self, _: SystemState, updated: SystemState):
        return

    def getBlur(self):
        blur = self.config.getInt(BLUR)
        blur = max(1, blur)
        return (blur, blur)

    def region_of_disinterest(self, img, vertices):
        mask = np.ones_like(img) * 255
        cv2.fillPoly(mask, vertices, 0)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def flatten_image(self, img):
        top_left = (int)(img.shape[1] * 0.26), (int)(img.shape[0])
        top_right = (int)(img.shape[1] * 0.74), (int)(img.shape[0])
        bottom_left = 0, 0
        bottom_right = (int)(img.shape[1]), 0

        src = np.float32([top_left, top_right, bottom_left, bottom_right])
        dst = np.float32(
            [[0, img.shape[0]], [img.shape[1], img.shape[0]], [0, 0], [img.shape[1], 0]])

        M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

    def generate_occupancy_map(self, img):
        # Resize and flatten
        datamap = cv2.resize(img, dsize=(
            80, 80), interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))

        msg = OccupancyGrid(info=g_mapData, data=flat)

        # Visualize using plt
        # plt.imshow(datamap, cmap='gray')
        # plt.show(block = False)
        # plt.pause(0.001)

        self.m_laneMapPublisher.publish(msg)

    def onImageReceived(self, image: CompressedImage):
        # Decompressify
        cv_image = g_bridge.compressed_imgmsg_to_cv2(image)

        # Blur it up
        for _ in range(self.config.getInt(BLUR_ITERATIONS)):
            cv_image = cv2.blur(cv_image, self.getBlur())

        # Apply filter and return a mask
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower = (
            self.config.getInt(LOWER_HUE),
            self.config.getInt(LOWER_SATURATION),
            self.config.getInt(LOWER_VALUE)
        )
        upper = (
            self.config.getInt(UPPER_HUE),
            self.config.getInt(UPPER_SATURATION),
            self.config.getInt(UPPER_VALUE)
        )
        mask = cv2.inRange(img, lower, upper)
        mask = 255 - mask
        mask[mask < 250] = 0

        # Apply region of disinterest and flattening
        height = img.shape[0]
        width = img.shape[1]
        region_of_disinterest_vertices=[
            (135, height),
            (width / 3, (height / 1.5) + self.config.getInt(REGION_OF_DISINTEREST_TL)),
            (width - (width / 3), (height / 1.5) + self.config.getInt(REGION_OF_DISINTEREST_TR)),
            (width - 135, height)
        ]
        mask = self.region_of_disinterest(mask, np.array([region_of_disinterest_vertices], np.int32))
        mask = self.flatten_image(mask)

        preview_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        cv2.polylines(preview_image, np.array([region_of_disinterest_vertices], np.int32), True, (0, 255, 0), 2)
        preview_msg = g_bridge.cv2_to_compressed_imgmsg(preview_image)
        preview_msg.header = image.header
        preview_msg.format = "jpeg"
        self.m_lanePreviewPublisher.publish(preview_msg)

        # Actually generate the map
        self.generate_occupancy_map(mask)


def main():
    rclpy.init()
    rclpy.spin(ImageTransformer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
