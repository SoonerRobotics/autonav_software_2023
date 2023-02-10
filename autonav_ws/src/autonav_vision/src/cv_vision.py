#!/usr/bin/env python3

import rclpy
import cv2

from sensor_msgs.msg._compressed_image import CompressedImage
from cv_bridge import CvBridge

def on_image_received(image: CompressedImage):
    bridge = CvBridge()
    cv_image = bridge.compressed_imgmsg_to_cv2(image)

    # Create a mask to filter out the white lanes
    img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_white = (0, 0, 35)
    upper_white = (255, 100, 170)
    mask = cv2.inRange(img, lower_white, upper_white)
    mask = 255 - mask
    
    # Show the image
    cv2.imshow("Robot Eyeballs - v1", mask)
    cv2.waitKey(1)

def main(args = None):
    rclpy.init()

    node = rclpy.create_node("autonav_cv_vision")
    node.create_subscription(CompressedImage, "/igvc/camera/compressed", on_image_received, 20)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()