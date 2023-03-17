#!/usr/bin/env python3

from autonav_msgs.msg import MotorInput, Position
from autonav_libs import AutoNode, Device, DeviceStateEnum, clamp
from nav_msgs.msg import Path
from pure_pursuit import PurePursuit
import math
import rclpy


class PathResolverNode(AutoNode):
    def __init__(self):
        super().__init__(Device.NAV_RESOLVER, "autonav_nav_resolver")

        self.m_position = Position()

    def setup(self):
        self.setDeviceState(DeviceStateEnum.READY)
        self.setDeviceState(DeviceStateEnum.OPERATING)

        self.m_pathSubscriber = self.create_subscription(Path, "/autonav/path", self.onPathReceived, 20)
        self.m_positionSubscriber = self.create_subscription(Position, "/autonav/position", self.onPositionReceived, 20)
        self.m_motorPublisher = self.create_publisher(MotorInput, "/autonav/MotorInput", 20)
        self.create_timer(0.1, self.onResolve)
        self.m_purePursuit = PurePursuit()
        self.m_backCount = -1

    def onPositionReceived(self, msg):
        self.m_position = msg
        self.m_position.theta = math.degrees(self.m_position.theta)
        if self.m_position.theta < 0:
            self.m_position.theta += 360

    def getAngleDifference(self, f, t):
        deltaAngle = t - f
        return (deltaAngle + math.pi) % (2 * math.pi) - math.pi

    def onPathReceived(self, msg: Path):
        self.points = [x.pose.position for x in msg.poses]
        self.m_purePursuit.set_points((point.x, point.y) for point in self.points)

    def onResolve(self):
        if self.getDeviceState() != DeviceStateEnum.OPERATING:
            return
        
        current = (self.m_position.x, self.m_position.y)
        lookahead = None
        radius = 0.7

        while lookahead is None and radius <= 4:
            lookahead = self.m_purePursuit.get_lookahead_point(current.x, current.y, radius)
            radius *= 1.2

        motorMsg = MotorInput()
        motorMsg.left_motor = 0
        motorMsg.right_motor = 0

        sortaDist = (lookahead[1] - current.y) ** 2 + (lookahead[0] - current.x) ** 2
        if self.m_backCount == -1 and (lookahead is not None and sortaDist > 0.1):
            wanted_heading = math.atan2(lookahead[1] - current.y, lookahead[0] - current.x)
            error = self.getAngleDifference(self.m_position.theta, wanted_heading) / math.pi
            speed = 0.9 * (1 - abs(error)) ** 5
            motorMsg.left_motor = (speed - clamp(0.5 * error, -0.25, 0.25))
            motorMsg.right_motor = (speed + clamp(0.5 * error, -0.25, 0.25))
        else:
            if self.m_backCount == 1:
                self.m_backCount = 5
            else:
                self.m_backCount -= 1

            motorMsg.left_motor = -0.35
            motorMsg.right_motor = -0.25

        self.m_motorPublisher.publish(motorMsg)

def main():
    rclpy.init()
    rclpy.spin(PathResolverNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
