#includes Orb feature detection implementation by github:niconielsen32 link: https://github.com/niconielsen32/VisualSLAM
#adapted for ROS 2 and Ubuntu 22 for VNAV course

import os
import cv2
import time
import numpy as np
import math
import rclpy
import time
import random
from rclpy.node import Node
from rclpy.publisher import Publisher

#for project
from matplotlib import pyplot as plt 
from lib.visualization.video import play_trip
from tqdm import tqdm

from cv_bridge import CvBridge
bridge = CvBridge()

image_publisher: Publisher = None 

class myNode(Node):

    def __init__(self):
        super().__init__("exampleNode")
        self.get_logger().info("Node ran")
        runExample()

def main(args=None):
    rclpy.init(args=args)

    node = myNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
	main()





