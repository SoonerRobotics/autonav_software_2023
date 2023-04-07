#!/usr/bin/env python3

from autonav_msgs.msg import MotorInput, Position
from autonav_libs import AutoNode, DeviceStateEnum, clamp
from nav_msgs.msg import Path
from pure_pursuit import PurePursuit
from pp_viewer import draw_pp
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
        self.m_idx = 0
        self.m_pathSubscriber = self.create_subscription(Path, "/autonav/path", self.onPathReceived, 20)
        self.m_positionSubscriber = self.create_subscription(Position, "/autonav/position", self.onPositionReceived, 20)
        self.m_motorPublisher = self.create_publisher(MotorInput, "/autonav/MotorInput", 20)
        self.create_timer(0.1, self.onResolve)

    def onPositionReceived(self, msg):
        self.m_position = msg
        self.m_position.theta = self.m_position.theta

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
        points = [(3, 0), (3, 3), (0, 3), (0, 0)]
        lookahead = points[self.m_idx]
        motor_pkt = MotorInput()
        motor_pkt.left_motor = 0.0
        motor_pkt.right_motor = 0.0

        # If we have a lookahead point, we can calculate the angle difference
        if lookahead is None:
            return
        
        angle_diff = self.getAngleDifference(math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0]), self.m_position.theta)
        dist = math.sqrt((lookahead[0] - cur_pos[0]) ** 2 + (lookahead[1] - cur_pos[1]) ** 2)

        if dist < 0.1:
            self.m_idx += 1
            if self.m_idx >= len(points):
                self.m_idx = 0

        # If we're within 5 degrees of the angle, we can go straight
        if abs(angle_diff) < math.radians(5):
            motor_pkt.left_motor = 1.0
            motor_pkt.right_motor = 1.0
        else:
            speed = 0.8 * (1 - abs(angle_diff) / math.pi)
            motor_pkt.left_motor = (speed - clamp(0.3 * angle_diff, -0.3, 0.3)) * clamp(0 if dist < 0.1 else dist, 0, 1.0)
            motor_pkt.right_motor = (speed + clamp(0.3 * angle_diff, -0.3, 0.3)) * clamp(0 if dist < 0.1 else dist, 0, 1.0)

        self.m_motorPublisher.publish(motor_pkt)


def main():
    rclpy.init()
    rclpy.spin(PathResolverNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
