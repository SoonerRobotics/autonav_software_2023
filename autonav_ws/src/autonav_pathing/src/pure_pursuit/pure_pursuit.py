#!/usr/bin/env python3

import lookahead_finder
import pursuit_test
import rclpy
from rclpy.node import Node
import random
import math
from autonav_libs import clamp
from autonav_msgs.msg import GoalPoint
from autonav_msgs.msg import Path
from autonav_msgs.msg import Position
from autonav_msgs.msg import MotorInput

class PurePursuit(Node):

    def __init__(self):
        super().__init__("pure_pursuit")
        self.publisher = self.create_publisher(GoalPoint, '/autonav/goal_point', 10)
        self.path_subscription = self.create_subscription(Path, '/autonav/Path', self.accept_path, 10)
        self.position_subscription = self.create_subscription(Position, '/autonav/position', self.accept_position, 10)
        self.motor_publisher = self.create_publisher(MotorInput, "/autonav/MotorInput", 20)
        self.create_timer(0.1, self.onResolve)
        self.purePursuit = lookahead_finder.PurePursuit()
        self.path_subscription
        self.robo_position = [0.0, 0.0, 0.0]
        self.local_path = []
        self.backCount = -1

    def accept_position(self, position = Position):
        #self.get_logger().info(f"hearing {position} from position topic")
        self.robo_position = [position.x, position.y, position.theta]
        # self.m_position.theta = self.m_position.theta
        self.robo_position[0] = 0.0
        self.robo_position[1] = 0.0
        self.robo_position[2] = 0.0

    def getAngleDifference(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi       
        return delta
    
    def get_lookahead(self, local_path):
        path = lookahead_finder.PurePursuit()
        path.setpath(local_path)
        lookahead = None
        radius = .7
        while lookahead == None and radius <= 4.0:
            lookahead = path.get_lookahead_point(local_path[0][0], local_path[0][1], radius)
        
        return lookahead
            
    def accept_path(self, msg):
        self.local_path = []
        path_data = msg.path_data
        for Waypoint in path_data:
            self.local_path.append([Waypoint.x, Waypoint.y])
        self.get_logger().info(f'I heard {self.local_path} as the local_path')
        self.purePursuit.setpath(self.local_path)
        
        #lookahead = self.get_lookahead(local_path)
        #path = lookahead_finder.PurePursuit()
        #path.setpath(local_path)
        #lookahead = path.get_lookahead_point(local_path[0][0], local_path[0][1], 2.3)
        #self.publish_lookahead(lookahead)
        #pursuit_test.pursuit_test(local_path[0], local_path, 2.3)
        
        
    def getAngleDifference(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi       
        return delta
    
    def onResolve(self):
        if self.purePursuit.path == []:
            return
        
        cur_pos = (self.robo_position[0], self.robo_position[1])
        lookahead = None
        radius = 0.7
        while lookahead is None and radius <= 4.0:
            lookahead = self.purePursuit.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
            pursuit_test.pursuit_test(cur_pos, self.local_path, radius)
            radius *= 1.2

        self.get_logger().info(f"Received lookahead -> {lookahead}")
        self.get_logger().info(f"The robots position is {self.robo_position}")
        
        motor_pkt = MotorInput()
        motor_pkt.forward_velocity = 0.0
        motor_pkt.angular_velocity = 0.0

        if lookahead is None:
            self.motor_publisher.publish(motor_pkt)
            return
        
        if self.backCount == -1 and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.1:  
            angle_diff = math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0])
            error = self.getAngleDifference(angle_diff, self.robo_position[2]) / math.pi
            forward_speed = 0.7 * (1 - abs(angle_diff)) ** 5
            motor_pkt.forward_velocity = forward_speed
            motor_pkt.angular_velocity = clamp(error * 2.0, -1.5, 1.5)  
        else:
            if self.backCount == -1:
                self.backCount = 5
            else:
                self.backCount -= -1

            motor_pkt.forward_velocity = -0.5
            motor_pkt.angular_velocity = 0.0
        
        self.motor_publisher.publish(motor_pkt)
        self.get_logger().info(f"publishing {motor_pkt} to /igvc/motors_raw")

    def publish_lookahead(self, lookahead):
        msg = GoalPoint()
        msg.goalpoint_x = lookahead[0]
        msg.goalpoint_y = lookahead[1]
        self.publisher.publish(msg)
        self.get_logger().info('Publishing goalpoint_x: "%f"' % msg.goalpoint_x)
        self.get_logger().info('Publishing goalpoint_y: "%f"' % msg.goalpoint_y)


def main(args=None):
   
    # test using random simulated position, path, and radius
    # test1 = get_random_pursuit_simulation()
    # pursuit_test.pursuit_test(test1[0], test1[1], test1[2])

    # hard coded 
    #waypoints = [[-4, -2], [-4, 2], [-2, 2], [-2, -2], [0, -2], [0, 2], [2, 2], [2, -2]]
    
    #path = lookahead_finder.PurePursuit()
    #path.setpath(waypoints)
    

    #lookahead = path.get_lookahead_point(0, 0, 2.3)


    # waypoint publisher node initiation
    rclpy.init(args=args)

    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)

    #pure_pursuit.publish_lookahead(lookahead)
    #pursuit_test.pursuit_test((0,0), waypoints, 2.3)
    pure_pursuit.destroy_node
    rclpy.shutdown

    

if __name__ == "__main__":
    main()
    
    