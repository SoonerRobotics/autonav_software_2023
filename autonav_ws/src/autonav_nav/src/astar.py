#!/usr/bin/env python3

from autonav_libs import AutoNode, Device, DeviceStateEnum
from nav_msgs.msg import OccupancyGrid
import rclpy


class AStarNode(AutoNode):
    def __init__(self):
        super().__init__(Device.NAV_ASTAR, "autonav_nav_astar")

        self.m_configSpace = None

    def setup(self):
        self.setDeviceState(DeviceStateEnum.READY)
        self.setDeviceState(DeviceStateEnum.OPERATING)

        self.m_configSpaceSubscriber = self.create_subscription(OccupancyGrid, "/autonav/cfg_space", self.onConfigSpaceReceived, 20)

    def onConfigSpaceReceived(self, msg: OccupancyGrid):
        self.m_configSpace = msg

def main():
    rclpy.init()
    rclpy.spin(AStarNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
