#!/usr/bin/env python3

from autonav_msgs.msg import MotorInput, Position
from autonav_libs import AutoNode, DeviceStateEnum, clamp
from nav_msgs.msg import Path
from pure_pursuit import PurePursuit
import math
import rclpy


class PathResolverNode(AutoNode):
    def __init__(self):
        super().__init__("autonav_nav_resolver")

        self.m_position = Position()

    def setup(self):
        self.setDeviceState(DeviceStateEnum.READY)
        self.setDeviceState(DeviceStateEnum.OPERATING)

        self.m_purePursuit = PurePursuit()
        self.backCount = -1
        self.m_pathSubscriber = self.create_subscription(Path, "/autonav/path", self.onPathReceived, 20)
        self.m_positionSubscriber = self.create_subscription(Position, "/autonav/position", self.onPositionReceived, 20)
        self.m_motorPublisher = self.create_publisher(MotorInput, "/autonav/MotorInput", 20)
        self.create_timer(0.1, self.onResolve)

    def onPositionReceived(self, msg):
        self.m_position = msg
        # self.m_position.theta = self.m_position.theta
        self.m_position.x = 0.0
        self.m_position.y = 0.0
        self.m_position.theta = 0.0


    def getAngleDifference(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi       
        return delta

    def onPathReceived(self, msg: Path):
        self.points = [x.pose.position for x in msg.poses]
        self.m_purePursuit.set_points([(point.x, point.y) for point in self.points])

    def onResolve(self):
        if self.m_position is None:
            return

        cur_pos = (self.m_position.x, self.m_position.y)
        lookahead = None
        radius = 0.7
        while lookahead is None and radius <= 4.0:
            lookahead = self.m_purePursuit.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
            radius *= 1.2
        
        motor_pkt = MotorInput()
        motor_pkt.forward_velocity = 0.0
        motor_pkt.angular_velocity = 0.0

        if lookahead is None:
            self.m_motorPublisher.publish(motor_pkt)
            return
        
        if self.backCount == -1 and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.1:  
            angle_diff = math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0])
            error = self.getAngleDifference(angle_diff, self.m_position.theta) / math.pi
            forward_speed = 0.7 * (1 - abs(angle_diff)) ** 5
            motor_pkt.forward_velocity = forward_speed
            motor_pkt.angular_velocity = clamp(error * 2.0, -1.5, 1.5)
        else:
            if self.backCount == -1:
                self.backCount = 5
            else:
                self.backCount -= -1

        self.m_motorPublisher.publish(motor_pkt)


def main():
    rclpy.init()
    rclpy.spin(PathResolverNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
