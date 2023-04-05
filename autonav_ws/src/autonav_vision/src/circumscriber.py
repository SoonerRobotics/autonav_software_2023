#!/usr/bin/env python3

import numpy as np
import cv2 as cv
from enum import IntEnum
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage

bridge = CvBridge()

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

class Circumscriber(Node):

    def __init__(self):
        super().__init__("circumscriber")
        self.subscriber = self.create_subscription(CompressedImage, "/igvc/camera/compressed", self.on_image_received, 10)
        self.publisher = self.create_publisher(CompressedImage, "/autonav/camera/filtered", 1)

    def on_image_received(self, image: CompressedImage):

        self.get_logger().info("Image received")
        # Decompress
        cv_image = bridge.compressed_imgmsg_to_cv2(image)

        # Blur it up
        for _ in range(1): # Register.BLUR_ITERATIONS
            cv_image = cv.blur(cv_image, self.get_blur())

        # Apply filter and return a mask
        img = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        lower = (
            0, # LOWER_HUE
            0, # LOWER_SATURATION
            35, #LOWER_VALUE
        )
        upper = (
            255, # UPPER_HUE
            100, # UPPER_SATURATION
            170, # UPPER_VALUE
        )
        mask = cv.inRange(img, lower, upper)
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
        if True: # Register.APPLY_FLATTENING
            map_image = self.region_of_interest(mask, np.array([region_of_interest_vertices], np.int32))
        
        if True: # Register.APPLY_REGION_OF_INTEREST
            map_image = self.flatten_image(mask)
        
        # Convert mask to RGB for preview
        preview_image = cv.cvtColor(mask, cv.COLOR_GRAY2RGB)
        cv.imshow("preview_image", preview_image)
        cv.waitKey(5000)
        preview_msg = bridge.cv2_to_compressed_imgmsg(preview_image)
        preview_msg.header = image.header
        preview_msg.format = "jpeg"
        self.publisher.publish(preview_msg)
        self.get_logger().info("Publishing an image")
        

    def get_blur(self):
        blur = 5 # Register.BLUR
        blur = max(1, blur)
        return (blur, blur)
    
    def region_of_interest(self, img, vertices):
        mask = np.zeros_like(img) * 255
        match_mask_color = 0
        cv.fillPoly(mask, vertices, match_mask_color)
        return cv.bitwise_and(img, mask)

    def flatten_image(self, img):
        top_left = (int)(img.shape[1] * 0.26), (int)(img.shape[0])
        top_right = (int)(img.shape[1] * 0.74), (int)(img.shape[0])
        bottom_left = 0, 0
        bottom_right = (int)(img.shape[1]), 0
        
        src = np.float32([top_left, top_right, bottom_left, bottom_right])
        dst = np.float32([[0, img.shape[0]], [img.shape[1], img.shape[0]], [0, 0], [img.shape[1], 0]])

        M = cv.getPerspectiveTransform(src, dst)
        return cv.warpPerspective(img, M, (img.shape[1], img.shape[0]))
    
    
def main(args=None):
    rclpy.init(args=args)

    circumscriber = Circumscriber()

    rclpy.spin(circumscriber)

    circumscriber.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
