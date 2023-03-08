#!/usr/bin/env python3

import rclpy
import cv2
from enum import IntEnum

from sensor_msgs.msg._compressed_image import CompressedImage
from cv_bridge import CvBridge
from autonav_libs import AutoNode, Device, DeviceStateEnum as DeviceState

g_bridge = CvBridge()

class Register(IntEnum):
    LOWER_HUE = 0
    LOWER_SATURATION = 1
    LOWER_VALUE = 2
    
    UPPER_HUE = 3
    UPPER_SATURATION = 4
    UPPER_VALUE = 5
    
    BLUR = 6
    BLUR_ITERATIONS = 7

class ImageTransformer(AutoNode):
    def __init__(self):
        super().__init__(Device.IMAGE_TRANSFORMER, "autonav_vision_transformer")
        
    def setup(self):
        self.config.writeInt(Register.LOWER_HUE, 0)
        self.config.writeInt(Register.LOWER_SATURATION, 0)
        self.config.writeInt(Register.LOWER_VALUE, 35)
        self.config.writeInt(Register.UPPER_HUE, 255)
        self.config.writeInt(Register.UPPER_SATURATION, 100)
        self.config.writeInt(Register.UPPER_VALUE, 170)
        self.config.writeInt(Register.BLUR, 5)
        self.config.writeInt(Register.BLUR_ITERATIONS, 1)
        
        self.log("setup?")
        
        self.m_cameraSubscriber = self.create_subscription(CompressedImage, "/igvc/camera/compressed", self.onImageReceived, 20)
        self.m_lanePreviewPublisher = self.create_publisher(CompressedImage, "/autonav/camera/filtered", 20)
        self.setDeviceState(DeviceState.READY)
        
    def getBlur(self):
        blur = self.config.readInt(Register.BLUR)
        blur = max(1, blur)
        return (blur, blur)

    def onImageReceived(self, image: CompressedImage):
        if self.getDeviceState() != DeviceState.OPERATING:
            return
        
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
        
        # Convert mask to RGB
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        
        preview_msg = g_bridge.cv2_to_compressed_imgmsg(mask)
        preview_msg.header = image.header
        preview_msg.format = "jpeg"
        self.m_lanePreviewPublisher.publish(preview_msg)

def main():
    rclpy.init()
    rclpy.spin(ImageTransformer())
    rclpy.shutdown()


if __name__ == "__main__":
    main()