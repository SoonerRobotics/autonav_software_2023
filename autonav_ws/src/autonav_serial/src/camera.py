#!/usr/bin/env python3

import rclpy
import time
import threading
import cv2
from vnpy import *
from enum import IntEnum

from autonav_libs import Device, AutoNode, DeviceStateEnum
from sensor_msgs.msg._image import Image

from cv_bridge import CvBridge

bridge = CvBridge()


class Register(IntEnum):
    REFRESH_RATE = 0


class Cameranode(AutoNode):
    def __init__(self):
        super().__init__(Device.CAMERA_TRANSLATOR, "autonav_serial_camera")
        self.config.writeInt(Register.REFRESH_RATE, 1)

    def setup(self):
        self.sensor = VnSensor()
        self.has_published_headers = False

        self.camera_publisher = self.create_publisher(
            Image, "/autonav/camera/raw", 20)
        self.camera_thread = threading.Thread(target=self.camera_read)
        self.camera_thread.start()
        self.set_device_state(DeviceStateEnum.OPERATING)

    def camera_read(self):
        cap = cv2.VideoCapture(0)
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret or frame is None:
                continue

            self.camera_publisher.publish(bridge.cv2_to_imgmsg(frame))
            time.sleep(1.0 / self.config.readInt(Register.REFRESH_RATE))


def main():
    rclpy.init()
    rclpy.spin(Cameranode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()