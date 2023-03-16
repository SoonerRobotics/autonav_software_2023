#!/usr/bin/env python3

import rclpy
import time
import threading
import cv2
from enum import IntEnum

from autonav_libs import Device, AutoNode, DeviceStateEnum, SystemStateEnum as SystemState, clamp
from sensor_msgs.msg._image import Image

from cv_bridge import CvBridge


REFRESH_RATE = 0
bridge = CvBridge()


class CameraNode(AutoNode):
    def __init__(self):
        super().__init__(Device.CAMERA_TRANSLATOR, "autonav_serial_camera")

        self.m_lastDeviceId = 0

    def setup(self):
        self.config.writeInt(REFRESH_RATE, 1)
        self.m_cameraPublisher = self.create_publisher(Image, "/autonav/camera/raw", 20)
        self.m_cameraThread = threading.Thread(target=self.camera_read)
        self.m_cameraThread.start()

    def shutdown(self):
        self.m_cameraThread.join()

    def camera_read(self):
        capture = None
        while rclpy.ok() and self.getDeviceState() != DeviceStateEnum.SHUTDOWN:
            try:
                capture = cv2.VideoCapture(clamp(self.m_lastDeviceId, 0, 10))
                if capture is None or not capture.isOpened():
                    self.m_lastDeviceId += 1
                    raise Exception("Could not open video device")

                self.setDeviceState(DeviceStateEnum.READY)
            except:
                self.setDeviceState(DeviceStateEnum.STANDBY)
                time.sleep(1.5)
                continue
                

            while rclpy.ok() and self.getDeviceState() != DeviceStateEnum.SHUTDOWN:
                if self.getDeviceState() != DeviceStateEnum.OPERATING:
                    continue

                try:
                    ret, frame = capture.read()
                except:
                    if capture is not None:
                        capture.release()
                        capture = None

                    self.setDeviceState(DeviceStateEnum.STANDBY)
                    break

                if not ret or frame is None:
                    continue

                self.m_cameraPublisher.publish(bridge.cv2_to_imgmsg(frame))
                time.sleep(1.0 / self.config.readInt(REFRESH_RATE))


def main():
    rclpy.init()
    rclpy.spin(CameraNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
