#!/usr/bin/env python3

import rclpy
import numpy as np
import matplotlib.pyplot as plt
from enum import IntEnum

from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, Point
from cv_bridge import CvBridge
from autonav_libs import AutoNode, DeviceStateEnum as DeviceState

g_bridge = CvBridge()

g_mapData = MapMetaData()
g_mapData.width = 100
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


class Expandifier(AutoNode):
    def __init__(self):
        super().__init__("autonav_vision_expandifier")

        self.verticalCameraRange = 2.75
        self.horizontalCameraRange = 3

        self.maxRange = 0.55
        self.noGoPercent = 0.75
        self.noGoRange = self.maxRange * self.noGoPercent
        
        self.maxRange = int(self.maxRange / (self.horizontalCameraRange / 80))
        self.noGoRange = int(self.noGoRange / (self.horizontalCameraRange / 80))

        self.metaData = MapMetaData(
            width=100,
            height=100,
            resolution=0.1,
            origin=Pose(
                position=Point(
                    x=-10.0,
                    y=-10.0
                )
            )
        )

    def setup(self):
        pts = list(range(-self.maxRange, self.maxRange + 1))
        self.circles = [(0, 0, 0)]
        for x in pts:
            for y in pts:
                if self.maxRange * self.noGoPercent <= np.sqrt(x ** 2 + y ** 2) < self.maxRange and (x + y) % 3 == 0:
                    self.circles.append((x, y, np.sqrt(x ** 2 + y ** 2)))

        self.m_rawConfigSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space/raw", self.onRawCfgSpaceReceived, 1)
        self.m_configPublisher = self.create_publisher(OccupancyGrid, "/autonav/cfg_space/expanded", 1)
        self.setDeviceState(DeviceState.READY)
        self.setDeviceState(DeviceState.OPERATING)
        
    def onRawCfgSpaceReceived(self, grid: OccupancyGrid):
        cfg_space = [0] * (80 * 80)
        for x in range(80):
            for y in range(1, 80):
                if grid.data[x + y * 80] > 0:
                    for x_i, y_i, dist in self.circles:
                            index = (x + x_i) + 80 * (y + y_i)

                            if 0 <= (x + x_i) < 80 and 0 <= (y + y_i) < 80:
                                val_at_index = cfg_space[index]
                                linear_t = 100 - ((dist - self.noGoRange) /
                                                    (self.maxRange - self.noGoRange) * 100)

                                if dist <= self.noGoRange:
                                    # obstacle expansion
                                    cfg_space[index] = 100
                                elif dist <= 100 and val_at_index <= linear_t:
                                    # linearly decay
                                    cfg_space[index] = int(linear_t)

        new_msg = OccupancyGrid(data=cfg_space, info=self.metaData)
        self.m_configPublisher.publish(new_msg)
        
        # Show it using matplotlib
        # plt.imshow(np.array(cfg_space).reshape(80, 80))
        # plt.show(block = False)
        # plt.pause(0.01)

def main():
    rclpy.init()
    rclpy.spin(Expandifier())
    rclpy.shutdown()


if __name__ == "__main__":
    main()