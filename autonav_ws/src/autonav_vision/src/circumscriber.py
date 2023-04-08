#!/usr/bin/env python3

import numpy as np
import cv2 as cv
from enum import IntEnum
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import time

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
        preview_image = mask
        cv.imshow("preview_image", preview_image)
        cv.waitKey(5000)


        start = time.time()
        ret, thresh = cv.threshold(preview_image, 150, 255, 0)
        # split the image into parts
        h, w = thresh.shape
        print(f"h = {h} w = {w}")

        # define how many sections of the image you want to search for objects in
        grid_sections = 4
        sections = grid_sections // 2
        fractional_w = w // sections
        fractional_h = h // sections
        grid = []

        # for each grid section, find the contours and find the circles
        # solve the grid problem
        for i in len(grid_sections):
            right_displacement = 0
            left_displacement = 0
            bottom_displacement = 0
            top_displacement = 0
            contours, _ = cv.findContours(thresh[fractional_w * right_displacement: fractional_w * left_displacement, fractional_h * bottom_displacement : fractional_h * top_displacement], cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)



        contours, _ = cv.findContours(thresh[:fractional_w, :fractional_h], cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        print(f"Number of objects detected: {len(contours)}")

        preview_image = cv.cvtColor(preview_image, cv.COLOR_GRAY2RGB)
        for cnt in contours:
            [x,y], radius = cv.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            radius = int(radius)
            self.get_logger().info(f"Circle info: center {x, y}, radius {radius}")
            preview_image = cv.circle(preview_image, center, radius, (0,0,255), 2)
        
        end = time.time()
        self.get_logger().info(f"Time to draw circles: {end - start}")
        # Display the image
        cv.imshow("preview_image_with_circles", preview_image)
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
