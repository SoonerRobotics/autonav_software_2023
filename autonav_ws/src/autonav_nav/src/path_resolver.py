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
        self.m_backCount = -1
        self.m_pathSubscriber = self.create_subscription(Path, "/autonav/path", self.onPathReceived, 20)
        self.m_positionSubscriber = self.create_subscription(Position, "/autonav/position", self.onPositionReceived, 20)
        self.m_motorPublisher = self.create_publisher(MotorInput, "/autonav/MotorInput", 20)
        self.create_timer(0.1, self.onResolve)

    def onPositionReceived(self, msg):
        self.m_position = msg
        self.m_position.theta = math.degrees(self.m_position.theta)
        if self.m_position.theta < 0:
            self.m_position.theta += 360

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
        radius = 0.7 # Starting radius

        while lookahead is None and radius <= 4: # Look until we hit 3 meters max
            lookahead = self.m_purePursuit.get_lookahead_point(cur_pos[0], cur_pos[1], radius)
            radius *= 1.2

        draw_pp(cur_pos, lookahead, self.m_purePursuit.path)

        motor_pkt = MotorInput()
        motor_pkt.left_motor = 0.0
        motor_pkt.right_motor = 0.0

        if self.m_backCount == -1 and (lookahead is not None and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.1):
            # Get heading to to lookahead from current position
            heading_to_lookahead = math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0])

            # print(f"h_2_l: {heading_to_lookahead * 180 / (math.pi)}")

            # Get difference in our heading vs heading to lookahead
            # Normalize error to -1 to 1 scale
            error = self.getAngleDifference(heading_to_lookahead, self.m_position.theta) / math.pi

            # print(f"am at {cur_pos[0]:0.02f},{cur_pos[1]:0.02f}, want to go to {lookahead[0]:0.02f},{lookahead[1]:0.02f}")
            # print(f"angle delta: {error * 180:0.01f}")

            # print(f"error is {error}")
            # if abs(error) < 2.0:
            #     error = 0

            # Base forward velocity for both wheels
            forward_speed = 0.9 * (1 - abs(error))**5

            # Define wheel linear velocities
            # Add proprtional error for turning.
            # TODO: PID instead of just P
            motor_pkt.left_motor = (forward_speed - clamp(0.5 * error, -0.25, 0.25))
            motor_pkt.right_motor = (forward_speed + clamp(0.5 * error, -0.25, 0.25))

        else:
            # We couldn't find a suitable direction to head, stop the robot.
            if self.m_backCount == -1:
                self.m_backCount = 5
            else:
                self.m_backCount -= 1
            
            motor_pkt.left_motor = -0.35
            motor_pkt.right_motor = -0.25

        self.m_motorPublisher.publish(motor_pkt)

def main():
    rclpy.init()
    rclpy.spin(PathResolverNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
