#!/usr/bin/env python3

from autonav_msgs.msg import MotorInput, Position
from scr_msgs.msg import SystemState
from scr_core.node import Node
from scr_core.state import DeviceStateEnum, SystemStateEnum
from scr_core import clamp
from nav_msgs.msg import Path
from pure_pursuit import PurePursuit
import math
import rclpy


FORWARD_SPEED = 0
REVERSE_SPEED = 1
RADIUS_MULTIPLIER = 2
RADIUS_MAX = 3
RADIUS_START = 4


class PathResolverNode(Node):
    def __init__(self):
        super().__init__("autonav_nav_resolver")
        self.position = Position()

    def configure(self):
        self.purePursuit = PurePursuit()
        self.backCount = -1
        self.pathSubscriber = self.create_subscription(Path, "/autonav/path", self.onPathReceived, 20)
        self.positionSubscriber = self.create_subscription(Position, "/autonav/position", self.onPositionReceived, 20)
        self.motorPublisher = self.create_publisher(MotorInput, "/autonav/MotorInput", 20)
        
        self.config.setFloat(FORWARD_SPEED, 0.75)
        self.config.setFloat(REVERSE_SPEED, -0.5)
        self.config.setFloat(RADIUS_MULTIPLIER, 1.2)
        self.config.setFloat(RADIUS_MAX, 4.0)
        self.config.setFloat(RADIUS_START, 0.7)
        
        self.create_timer(0.1, self.onResolve)
        self.setDeviceState(DeviceStateEnum.READY)

    def onReset(self):
        self.position = None
        self.backCount = -1

    def transition(self, old: SystemState, updated: SystemState):
        if updated.state == SystemStateEnum.AUTONOMOUS and self.getDeviceState() == DeviceStateEnum.READY:
            self.setDeviceState(DeviceStateEnum.OPERATING)
            
        if updated.state != SystemStateEnum.AUTONOMOUS and self.getDeviceState() == DeviceStateEnum.OPERATING:
            self.setDeviceState(DeviceStateEnum.READY)
            inputPacket = MotorInput()
            inputPacket.forward_velocity = 0.0
            inputPacket.angular_velocity = 0.0
            self.motorPublisher.publish(inputPacket)

    def onPositionReceived(self, msg):
        self.position = msg
        self.position.x = 0.0
        self.position.y = 0.0
        self.position.theta = 0.0

    def getAngleDifference(self, to_angle, from_angle):
        delta = to_angle - from_angle
        delta = (delta + math.pi) % (2 * math.pi) - math.pi
        return delta

    def onPathReceived(self, msg: Path):
        self.points = [x.pose.position for x in msg.poses]
        self.purePursuit.set_points([(point.x, point.y) for point in self.points])

    def onResolve(self):
        if self.position is None or self.getDeviceState() != DeviceStateEnum.OPERATING or self.getSystemState().state != SystemStateEnum.AUTONOMOUS:
            return

        cur_pos = (self.position.x, self.position.y)
        lookahead = None
        radius = self.config.getFloat(RADIUS_START)
        while lookahead is None and radius <= self.config.getFloat(RADIUS_MAX):
            lookahead = self.purePursuit.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
            radius *= self.config.getFloat(RADIUS_MULTIPLIER)

        inputPacket = MotorInput()
        inputPacket.forward_velocity = 0.0
        inputPacket.angular_velocity = 0.0

        if lookahead is None:
            self.motorPublisher.publish(inputPacket)
            return

        if self.backCount == -1 and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.1:
            angle_diff = math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0])
            error = self.getAngleDifference(angle_diff, self.position.theta) / math.pi
            forward_speed = self.config.getFloat(FORWARD_SPEED) * (1 - abs(angle_diff)) ** 8
            inputPacket.forward_velocity = forward_speed
            inputPacket.angular_velocity = clamp(error * 3.0, -2.0, 2.0)
        else:
            if self.backCount == -1:
                self.backCount = 5
            else:
                self.backCount -= 1

            inputPacket.forward_velocity = self.config.getFloat(REVERSE_SPEED)
            inputPacket.angular_velocity = 0.0

        if not self.getSystemState().mobility:
            inputPacket.forward_velocity = 0.0
            inputPacket.angular_velocity = 0.0

        self.motorPublisher.publish(inputPacket)


def main():
    rclpy.init()
    rclpy.spin(PathResolverNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
