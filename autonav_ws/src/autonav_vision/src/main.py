import cv2
import time
import numpy as np
import math
import rclpy

FRAME_RATE_ = 10.0


if __name__ == '__main__':
    rclpy.create_node('lane_finder')

    cam = cv2.VideoCapture(2, cv2.CAP_V4L2)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    #rclpy.Timer(rclpy.Duration(1.0/FRAME_RATE_), camera_callback) TODO: camera_callback

    rclpy.spin()





