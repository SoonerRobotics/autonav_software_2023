#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from enum import IntEnum

from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Pose, Point
from cv_bridge import CvBridge
from autonav_libs import AutoNode, Device, DeviceStateEnum as DeviceState

g_bridge = CvBridge()

g_mapData = MapMetaData()
g_mapData.width = 200
g_mapData.height = 100
g_mapData.resolution = 0.1
g_mapData.origin = Pose()
g_mapData.origin.position.x = -10.0
g_mapData.origin.position.y = -10.0

class Register(IntEnum):
    LOWER_HUE = 0
    LOWER_SATURATION = 1
    LOWER_VALUE = 2
    UPPER_HUE = 3
    UPPER_SATURATION = 4
    UPPER_VALUE = 5
    BLUR = 6
    BLUR_ITERATIONS = 7
    APPLY_FLATTENING = 8
    APPLY_REGION_OF_INTEREST = 9

class ImageTransformer(AutoNode):
    def __init__(self):
        super().__init__(Device.IMAGE_TRANSFORMER, "autonav_vision_transformer")

        self.camVert = 2.75
        self.camHoriz = 3

        self.maxRange = int(0.55 / (self.camHoriz / 80))
        self.badPercent = 0.75
        self.badRange = int((self.maxRange * self.badPercent) / (self.camHoriz / 80))

        self.metaData = MapMetaData(
            width = 100,
            height = 100,
            resolution = 0.1,
            origin = Pose(
                position = Point(
                    x = -10.0,
                    y = -10.0
                )
            )
        )
        
    def setup(self):
        self.config.writeInt(Register.LOWER_HUE, 0)
        self.config.writeInt(Register.LOWER_SATURATION, 0)
        self.config.writeInt(Register.LOWER_VALUE, 35)
        self.config.writeInt(Register.UPPER_HUE, 255)
        self.config.writeInt(Register.UPPER_SATURATION, 100)
        self.config.writeInt(Register.UPPER_VALUE, 170)
        self.config.writeInt(Register.BLUR, 5)
        self.config.writeInt(Register.BLUR_ITERATIONS, 1)
        self.config.writeBool(Register.APPLY_FLATTENING, True)
        self.config.writeBool(Register.APPLY_REGION_OF_INTEREST, True)

        pts = list(range(-self.maxRange, self.maxRange + 1))
        self.circles = [(0, 0, 0)]
        for x in pts:
            for y in pts:
                if self.maxRange * self.badPercent <= np.sqrt(x ** 2 + y ** 2) < self.maxRange and (x + y) % 3 == 0:
                    self.circles.append((x, y, np.sqrt(x ** 2 + y ** 2)))

        self.m_cameraSubscriber = self.create_subscription(CompressedImage, "/igvc/camera/compressed", self.onImageReceived, 20)
        self.m_laneMapPublisher = self.create_publisher(OccupancyGrid, "/autonav/map", 20)
        self.m_lanePreviewPublisher = self.create_publisher(CompressedImage, "/autonav/camera/filtered", 20)
        self.setDeviceState(DeviceState.READY)
        
    def getBlur(self):
        blur = self.config.readInt(Register.BLUR)
        blur = max(1, blur)
        return (blur, blur)

    def region_of_interest(self, img, vertices):
        mask = np.zeros_like(img) * 255
        match_mask_color = 0
        cv2.fillPoly(mask, vertices, match_mask_color)
        return cv2.bitwise_and(img, mask)
    
    def flatten_image(self, img):
        top_left = (int)(img.shape[1] * 0.26), (int)(img.shape[0])
        top_right = (int)(img.shape[1] * 0.74), (int)(img.shape[0])
        bottom_left = 0, 0
        bottom_right = (int)(img.shape[1]), 0
        
        src = np.float32([top_left, top_right, bottom_left, bottom_right])
        dst = np.float32([[0, img.shape[0]], [img.shape[1], img.shape[0]], [0, 0], [img.shape[1], 0]])

        M = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    
    def generate_occupancy_map(self, img):
        # Resize and flatten
        datamap = cv2.resize(img, (80, 80), interpolation=cv2.INTER_LINEAR) / 2
        flat = list(datamap.flatten().astype(int))
        
        msg = OccupancyGrid(info = g_mapData, data = flat)
        self.m_laneMapPublisher.publish(self.expandify_grid(msg))

    def expandify_grid(self, grid: OccupancyGrid):
        cfg_space = [0] * (80 * 80)
        for x in range(80):
            for y in range(80):
                if grid.data[x + y * 80] <= 0:
                    continue

                for x_i, y_i, dist in self.circles:
                    idx = (x + x_i) + 80 * (y + y_i)

                    if 0 <= (x + x_i) < 80 and 0 <= (y_i + y) < 80:
                        val = cfg_space[idx]
                        linear = 100 - (((dist - self.badRange) / (self.maxRange - self.badRange)) * 100)

                        if dist <= self.badRange:
                            cfg_space[idx] = 100
                        elif dist <= 100 and val <= linear:
                            cfg_space[idx] = int(linear)
        
        # self.m_visualization = np.zeros((80, 80, 3), np.uint8)
        # for x in range(80):
        #     for y in range(80):
        #         if cfg_space[x + y * 80] > 0:
        #             cv2.circle(self.m_visualization, (x, y), 1, (0, 0, 255), -1)
        #         else:
        #             cv2.circle(self.m_visualization, (x, y), 1, (0, 255, 0), -1)

        # cv2.imshow("Visualization", self.m_visualization)
        # cv2.waitKey(1)

        new_msg = OccupancyGrid(data = cfg_space, info = self.metaData)
        return new_msg

    def onImageReceived(self, image: CompressedImage):
        if self.getDeviceState() != DeviceState.OPERATING:
            return
        
        # Decompressify
        cv_image = g_bridge.compressed_imgmsg_to_cv2(image)

        # Blur it up
        for _ in range(self.config.readInt(Register.BLUR_ITERATIONS)):
            cv_image = cv2.blur(cv_image, self.getBlur())

        # Apply filter and return a mask
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower = (
            self.config.readInt(Register.LOWER_HUE),
            self.config.readInt(Register.LOWER_SATURATION),
            self.config.readInt(Register.LOWER_VALUE)
        )
        upper = (
            self.config.readInt(Register.UPPER_HUE),
            self.config.readInt(Register.UPPER_SATURATION),
            self.config.readInt(Register.UPPER_VALUE)
        )
        mask = cv2.inRange(img, lower, upper)
        mask = 255 - mask
        mask[mask < 250] = 0
        
        # Apply region of interest and flattening
        height = img.shape[0]
        width = img.shape[1]
        region_of_interest_vertices = [
            (0, height),
            (width / 2, height / 2 + 120),
            (width, height),
        ]
        
        map_image = mask
        if self.config.readBool(Register.APPLY_REGION_OF_INTEREST):
            map_image = self.region_of_interest(mask, np.array([region_of_interest_vertices], np.int32))
        
        if self.config.readBool(Register.APPLY_FLATTENING):
            map_image = self.flatten_image(mask)
        
        # Convert mask to RGB for preview
        preview_image = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        preview_msg = g_bridge.cv2_to_compressed_imgmsg(preview_image)
        preview_msg.header = image.header
        preview_msg.format = "jpeg"
        self.m_lanePreviewPublisher.publish(preview_msg)
        
        # Actually generate the map
        self.generate_occupancy_map(map_image)

def main():
    rclpy.init()
    rclpy.spin(ImageTransformer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()