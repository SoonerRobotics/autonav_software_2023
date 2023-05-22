#!/usr/bin/env python3

from scr_msgs.msg import SystemState
from scr_core.node import Node
from scr_core.state import DeviceStateEnum
import rclpy


CONFIG_RECORD_IMU = 0
CONFIG_RECORD_GPS = 1
CONFIG_RECORD_POSITION = 2
CONFIG_RECORD_FEEDBACK = 3
CONFIG_RECORD_OBJECTDETECTION = 4
CONFIG_RECORD_CAMERA = 5
CONFIG_RECORD_THRESHOLDED = 6
CONFIG_RECORD_EXPANDIFIED = 7
CONFIG_RECORD_MANUAL = 8
CONFIG_RECORD_AUTONOMOUS = 9


class PlaybackNode(Node):
    def __init__(self):
        super().__init__("autonav_playback")

    def configure(self):
        self.config.setBool(CONFIG_RECORD_IMU, True)
        self.config.setBool(CONFIG_RECORD_GPS, True)
        self.config.setBool(CONFIG_RECORD_POSITION, True)
        self.config.setBool(CONFIG_RECORD_FEEDBACK, True)
        self.config.setBool(CONFIG_RECORD_OBJECTDETECTION, True)
        self.config.setBool(CONFIG_RECORD_CAMERA, True)
        self.config.setBool(CONFIG_RECORD_THRESHOLDED, True)
        self.config.setBool(CONFIG_RECORD_EXPANDIFIED, True)
        self.config.setBool(CONFIG_RECORD_MANUAL, False)
        self.config.setBool(CONFIG_RECORD_AUTONOMOUS, True)
        
        self.setDeviceState(DeviceStateEnum.OPERATING)

    def transition(self, old: SystemState, updated: SystemState):
        pass


def main():
    rclpy.init()
    rclpy.spin(PlaybackNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
