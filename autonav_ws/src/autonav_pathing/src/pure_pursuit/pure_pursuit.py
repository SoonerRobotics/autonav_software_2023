#!/usr/bin/env python3

import lookahead_finder
import pursuit_test
import rclpy
from rclpy.node import Node
import random
from autonav_msgs.msg import Waypoint


class WaypointPublisher(Node):

    def __init__(self):
        super().__init__("waypoint_publisher")
        self.publisher = self.create_publisher(Waypoint, '/autonav/waypoints', 10)

    def publish_lookahead(self, lookahead):
        msg = Waypoint()
        msg.waypoint_x = lookahead[0]
        msg.waypoint_y = lookahead[1]
        self.publisher.publish(msg)
        self.get_logger().info('Publishing waypoint_x: "%f"' % msg.waypoint_x)
        self.get_logger().info('Publishing waypoint_y: "%f"' % msg.waypoint_y)


# returns a random robot position, random set of waypoints, and random radius
def get_random_pursuit_simulation():
    rand_wps = []
    rand_path_length = random.randint(4, 10)
    for i in range(rand_path_length):
        rand_x = random.randint(-5, 5)
        rand_y = random.randint(-5, 5)
        rand_wps.append((rand_x, rand_y))
    rand_robo_pos = random.randint(-2,2), random.randint(-2,2)
    rand_r = random.uniform(.5, 3)

    return rand_robo_pos, rand_wps, rand_r

def main(args = None):
   
    # test using random simulated position, path, and radius
    # test1 = get_random_pursuit_simulation()
    # pursuit_test.pursuit_test(test1[0], test1[1], test1[2])

    # hard coded 
    waypoints = [(-4, -2), (-4, 2), (-2, 2), (-2, -2), (0, -2), (0, 2), (2, 2), (2, -2)]
    
    path = lookahead_finder.PurePursuit()
    path.setpath(waypoints)
    

    lookahead = path.get_lookahead_point(0, 0, 2.3)


    # waypoint publisher node initiation
    rclpy.init(args=args)

    waypoint_publisher = WaypointPublisher()

    waypoint_publisher.publish_lookahead(lookahead)
    pursuit_test.pursuit_test((0,0), waypoints, 2.3)
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node
    rclpy.shutdown

    

if __name__ == "__main__":
    main()
    
    