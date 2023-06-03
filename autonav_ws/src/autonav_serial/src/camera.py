#!/usr/bin/env python3

import rclpy
import time
import threading
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from scr_core.node import Node
from scr_core.state import DeviceStateEnum, SystemStateEnum
import os

REFRESH_RATE = "refresh_rate"
OUTPUT_WIDTH = "output_width"
OUTPUT_HEIGHT = "output_height"
CAMERA_INDEX = "camera_index"

bridge = CvBridge()


class CameraNode(Node):
    def __init__(self):
        super().__init__("autonav_serial_camera")

    def configure(self):
        self.config.setInt(REFRESH_RATE, 8)
        self.config.setInt(OUTPUT_WIDTH, 640)
        self.config.setInt(OUTPUT_HEIGHT, 480)
        self.config.setInt(CAMERA_INDEX, 0)

        self.cameraPublisher = self.create_publisher(CompressedImage, "/autonav/camera/compressed", 20)
        self.cameraThread = threading.Thread(target=self.cameraWorker)
        self.cameraThread.daemon = True
        self.cameraThread.start()

    def transition(self, old, updated):
        return

    def cameraWorker(self):
        capture = None
        while rclpy.ok() and self.getSystemState().state != SystemStateEnum.SHUTDOWN:
            try:
                if not os.path.exists("/dev/video" + str(self.config.getInt(CAMERA_INDEX))):
                    time.sleep(1.5)
                    self.log("Could not find camera /dev/video" + str(self.config.getInt(CAMERA_INDEX)))
                    continue

                capture = cv2.VideoCapture(0)
                if capture is None or not capture.isOpened():
                    self.log("Could not open camera /dev/video" + str(self.config.getInt(CAMERA_INDEX)))
                    time.sleep(1.5)
                    continue

                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.getInt(OUTPUT_WIDTH))
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.getInt(OUTPUT_HEIGHT))
                self.setDeviceState(DeviceStateEnum.OPERATING)
            except:
                self.setDeviceState(DeviceStateEnum.STANDBY)
                time.sleep(1.5)
                continue

            while rclpy.ok() and self.getSystemState().state != SystemStateEnum.SHUTDOWN:
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

                self.cameraPublisher.publish(bridge.cv2_to_compressed_imgmsg(frame))
                time.sleep(1.0 / self.config.getInt(REFRESH_RATE))


def main():
    rclpy.init()
    rclpy.spin(CameraNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
