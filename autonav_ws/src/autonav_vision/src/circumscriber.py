#!/usr/bin/env python3

import numpy as np
import cv2 as cv
from enum import IntEnum
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import time
import math

from sensor_msgs.msg import CompressedImage
from autonav_msgs.msg import Obstacle 
from autonav_msgs.msg import Obstacles

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
        self.obstacle_publisher = self.create_publisher(Obstacles, "/autonav/obstacles", 1)
        self.h = None
        self.w = None


    def publish_obstacles(self, local_obstacle_list):
        msg = Obstacles()
        for local_obstacles in local_obstacle_list:
            obstacle = Obstacle()
            if local_obstacles[1] >= 240:
                obstacle.center_x, obstacle.center_y, obstacle.radius = local_obstacles[0], local_obstacles[1] - 2 * (local_obstacles[1] - 240), local_obstacles[2]
            else:
                obstacle.center_x, obstacle.center_y, obstacle.radius = local_obstacles[0], local_obstacles[1] + 2 * (240 - local_obstacles[1]), local_obstacles[2]
            
            msg.obstacles_data.append(obstacle)
        
        self.obstacle_publisher.publish(msg)
        self.get_logger().info(f"publishing {msg} as Obstacles to /autonav/obstacles")

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
        preview_image = map_image
        cv.imshow("preview_image", preview_image)
        cv.waitKey(1000)


        start = time.time()
        ret, thresh = cv.threshold(preview_image, 150, 255, 0)
        # split the image into parts
        h, w = thresh.shape
        self.h, self.w = h, w
        print(f"h = {h} w = {w}")

        # define how many sections of the image you want to search for objects in
        # grid sizes need to be square numbers
        
        sections = 4
        grid_sections = sections ** 2
        fractional_w = int(w // sections)
        fractional_h = int(h // sections)

        # for each grid section, find the contours and find the circles
        # solve the grid problem
        contour_collections = []
        for idx in range(grid_sections):
            right_displacement = int(idx % sections)
            left_displacement = int(sections - right_displacement - 1)
            bottom_displacement = int(idx // sections)
            top_displacement = int(sections - bottom_displacement - 1)
            contours, _ = cv.findContours(thresh[fractional_h * bottom_displacement: h - (fractional_h * top_displacement), fractional_w * right_displacement : w - (fractional_w * left_displacement)], cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            contour_collections.append(contours)


        obstacles = []
        preview_image = cv.cvtColor(preview_image, cv.COLOR_GRAY2RGB)
        # for the contours found, add them to the image
        for collections in range(len(contour_collections)):
            right_displacement = collections % sections
            left_displacement = sections - right_displacement -1
            bottom_displacement = collections // sections
            top_displacement = sections - bottom_displacement - 1

            for cnt in contour_collections[collections]:
                [x,y], radius = cv.minEnclosingCircle(cnt)
                center = (int(x) + int(fractional_w * right_displacement), int(y) + int(fractional_h * bottom_displacement))
                #center = (int(x), int(y))
                radius = int(radius)
                obstacles.append([center[0], center[1], radius])
                preview_image = cv.circle(preview_image, center, radius, (0,0,255), 2)
                obstacles.append((center[0], center[1], radius))

        # for testing whole or partial images
        end = time.time()
        self.get_logger().info(f"Time to draw circles: {end - start}")

        # display the image 
        cv.imshow("preview_image after circles", preview_image)
        cv.waitKey(10000)
        cv.destroyAllWindows()
        
        # send the obstacles to the path planner
        self.publish_obstacles(obstacles)
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
        top_left = (int)(img.shape[1] * 0), (int)(img.shape[0] * 1)
        top_right = (int)(img.shape[1] * 1), (int)(img.shape[0] * 1) 
        bottom_left = (int)(img.shape[1] * 0), (int)(img.shape[0] * 0)
        bottom_right = (int)(img.shape[1] * 1), (int)(img.shape[0] * 0)
        
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
