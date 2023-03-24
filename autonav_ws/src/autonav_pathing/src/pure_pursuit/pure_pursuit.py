#!/usr/bin/env python3

import lookahead_finder
import pursuit_test
import rclpy
from rclpy.node import Node
import random
from autonav_msgs.msg import GoalPoint
from autonav_msgs.msg import Path

class PathListener(Node):

    def __init__(self):
        super().__init__("path_listener")
        self.subscription = self.create_subscription(Path, '/autonav/Path', self.accept_path, 10)
        self.subscription

    def accept_path(self, msg):
        local_path = []
        path_data = msg.path_data
        for Waypoint in path_data:
            print(f"{Waypoint.x}, {Waypoint.y}")
            local_path.append([Waypoint.x, Waypoint.y])
        
        self.get_logger().info(f'I heard {local_path} as the local_path')


class GoalPointPublisher(Node):

    def __init__(self):
        super().__init__("goalpoint_publisher")
        self.publisher = self.create_publisher(GoalPoint, '/autonav/goal_point', 10)

    def publish_lookahead(self, lookahead):
        msg = GoalPoint()
        msg.goalpoint_x = lookahead[0]
        msg.goalpoint_y = lookahead[1]
        self.publisher.publish(msg)
        self.get_logger().info('Publishing goalpoint_x: "%f"' % msg.goalpoint_x)
        self.get_logger().info('Publishing goalpoint_y: "%f"' % msg.goalpoint_y)


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
    waypoints = [[-4, -2], [-4, 2], [-2, 2], [-2, -2], [0, -2], [0, 2], [2, 2], [2, -2]]
    
    path = lookahead_finder.PurePursuit()
    path.setpath(waypoints)
    

    lookahead = path.get_lookahead_point(0, 0, 2.3)


    # waypoint publisher node initiation
    rclpy.init(args=args)

    path_listener = PathListener()
    waypoint_publisher = GoalPointPublisher()

    waypoint_publisher.publish_lookahead(lookahead)
    pursuit_test.pursuit_test((0,0), waypoints, 2.3)
    rclpy.spin(path_listener)
    rclpy.spin(waypoint_publisher)
    path_listener.destroy_node
    waypoint_publisher.destroy_node
    rclpy.shutdown

    

if __name__ == "__main__":
    main()
    
    