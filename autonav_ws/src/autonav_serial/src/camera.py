#!/usr/bin/env python3

import rclpy
import time
import threading
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from scr_core.node import Node
from scr_core.state import DeviceStateEnum, SystemStateEnum

REFRESH_RATE = 0
bridge = CvBridge()


class CameraNode(Node):
    def __init__(self):
        super().__init__("autonav_serial_camera")

    def configure(self):
        self.config.setInt(REFRESH_RATE, 15)

        self.declare_parameter("device_id", 2)
        self.deviceId = self.get_parameter("device_id").value

        self.m_cameraPublisher = self.create_publisher(CompressedImage, "/autonav/camera/compressed", 20)
        self.m_cameraThread = threading.Thread(target=self.camera_read)
        self.m_cameraThread.start()

    def camera_read(self):
        capture = None
        while rclpy.ok() and self.getSystemState() != SystemStateEnum.SHUTDOWN:
            try:
                capture = cv2.VideoCapture(self.deviceId)
                if capture is None or not capture.isOpened():
                    raise Exception("Could not open video device")

                capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.setDeviceState(DeviceStateEnum.READY)
                self.setDeviceState(DeviceStateEnum.OPERATING)
            except:
                self.setDeviceState(DeviceStateEnum.STANDBY)
                time.sleep(1.5)
                continue
                

            while rclpy.ok() and self.getSystemState() != SystemStateEnum.SHUTDOWN:
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

                self.m_cameraPublisher.publish(bridge.cv2_to_compressed_imgmsg(frame))
                time.sleep(1.0 / self.config.readInt(REFRESH_RATE))


def main():
    rclpy.init()
    rclpy.spin(CameraNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
