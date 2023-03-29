#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from autonav_msgs.msg import Waypoint
from autonav_msgs.msg import Path



class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")
        self.publisher = self.create_publisher(Path, '/autonav/Path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

    def set_path(self, local_path):
        self.msg = Path()
        for local_waypoints in local_path:
            waypoint = Waypoint()
            waypoint.x, waypoint.y, waypoint.is_generated, waypoint.can_be_deleted = local_waypoints
            self.msg.path_data.append(waypoint)

    def publish_path(self):
        self.publisher.publish(self.msg)
        self.get_logger().info(f"Publishing {self.msg} as path to /autonav/Path")
        

# returns a random robot position, random set of waypoints, and random radius
def get_random_path():
    rand_wps = []
    rand_path_length = random.randint(4, 10)
    for i in range(rand_path_length):
        rand_x = float(random.randint(-5, 5))
        rand_y = float(random.randint(-5, 5))
        rand_wps.append([rand_x, rand_y, 0, 0])

    return rand_wps

def main(args = None):
    generated_path = get_random_path()

    rclpy.init(args=args)
    
    path_publisher = PathPublisher()
    path_publisher.set_path(generated_path)
    rclpy.spin(path_publisher)

    

    path_publisher.destroy_node
    rclpy.shutdown

if __name__ == "__main__":
    main()    


    