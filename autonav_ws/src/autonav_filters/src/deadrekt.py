#!/usr/bin/env python3

from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
import math
import rclpy
from rclpy.node import Node

class DeadReckoningFilter(Node):
    def __init__(self):
        super().__init__("dead_reckoning")
        self.position_publisher = self.create_publisher(Position, '/autonav/position', 10)
        self.feedback_subscriber = self.create_subscription(MotorFeedback, '/autonav/MotorFeedback', self.updateMotors, 10)
        self.reset()
        
    def reset(self):
        self.xSum = 0
        self.ySum = 0
        self.thetaSum = 0.0
        self.lastLat = None
        self.lastLong = None
        self.recordedPoints = []

    def updateMotors(self, feedback: MotorFeedback):
        self.get_logger().info(f"hearing {feedback} from velocity")
        self.xSum = self.xSum + feedback.delta_x * math.cos(self.thetaSum) + feedback.delta_y * math.sin(self.thetaSum)
        self.ySum = self.ySum + feedback.delta_x * math.sin(self.thetaSum) + feedback.delta_y * math.cos(self.thetaSum)
        self.thetaSum += feedback.delta_theta
        self.estimate()

    def updateGPS(self, msg: GPSFeedback):
        if msg.gps_fix <= 0 and msg.is_locked == False:
            return

        self.lastLat = msg.latitude
        self.lastLong = msg.longitude
        self.estimate()

    def estimate(self):
        msg = Position()
        offset = 1
        if True: #self.node.m_isSimulator
            offset = 10
        msg.x = self.xSum / offset
        msg.y = self.ySum / offset
        msg.theta = self.thetaSum
        if self.lastLat is not None and self.lastLong is not None:
            msg.latitude = self.lastLat
            msg.longitude = self.lastLong

        self.position_publisher.publish(msg)
        self.get_logger().info(f'publishing {msg} as the position')


def main(args=None):
    rclpy.init(args=args)

    dead_reckoning_filter = DeadReckoningFilter()

    rclpy.spin(dead_reckoning_filter)

    dead_reckoning_filter.destroy_node

    rclpy.shutdown()


if __name__ == "__main__":
    main()