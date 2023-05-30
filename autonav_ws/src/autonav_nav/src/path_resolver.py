#!/usr/bin/env python3

from autonav_msgs.msg import MotorInput, Position, SafetyLights
from scr_msgs.msg import SystemState
from scr_core.node import Node
from scr_core.state import DeviceStateEnum, SystemStateEnum
from scr_core import clamp
from nav_msgs.msg import Path
from pure_pursuit import PurePursuit
import math
import rclpy


FORWARD_SPEED = "forward_speed"
REVERSE_SPEED = "reverse_speed"
RADIUS_MULTIPLIER = "radius_multiplier"
RADIUS_MAX = "radius_max"
RADIUS_START = "radius_start"
ANGULAR_AGGRESSINON = "angular_aggression"
MAX_ANGULAR_SPEED = "max_angular_speed"

TURN_TYPE = "turn_type"

MIN_TURN_ANGLE = "min_turn_angle"
MIN_TURN_SPEED = "min_turn_speed"
TURN_COEFFICIENT = "turn_coefficient"

COLON_STRAT_A = "colon_strat_a"
COLON_STRAT_B = "colon_strat_b"

def hexToRgb(color: str):
    if color[0] == "#":
        color = color[1:]
    return [int(color[0:2], 16), int(color[2:4], 16), int(color[4:6], 16)]


def toSafetyLights(autonomous: bool, eco: bool, mode: int, brightness: int, color: str) -> SafetyLights:
    pkg = SafetyLights()
    pkg.mode = mode
    pkg.autonomous = autonomous
    pkg.eco = eco
    pkg.brightness = brightness
    colorr = hexToRgb(color)
    pkg.red = colorr[0]
    pkg.green = colorr[1]
    pkg.blue = colorr[2]
    return pkg


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
        self.safetyLightsPublisher = self.create_publisher(SafetyLights, "/autonav/SafetyLights", 20)
        
        self.config.setFloat(FORWARD_SPEED, 0.75)
        self.config.setFloat(REVERSE_SPEED, -0.5)
        self.config.setFloat(RADIUS_MULTIPLIER, 1.2)
        self.config.setFloat(RADIUS_MAX, 4.0)
        self.config.setFloat(RADIUS_START, 0.7)
        self.config.setFloat(ANGULAR_AGGRESSINON, 3.0)
        self.config.setFloat(MAX_ANGULAR_SPEED, 1.15)

        self.config.setFloat(TURN_TYPE, 1)

        self.config.setFloat(MIN_TURN_ANGLE, 0.5)
        self.config.setFloat(MIN_TURN_SPEED, 0.4)
        self.config.setFloat(TURN_COEFFICIENT, 1)

        self.config.setFloat(COLON_STRAT_A, 50.0)
        self.config.setFloat(COLON_STRAT_B, 0.2)
        
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
            
        if updated.state == SystemStateEnum.AUTONOMOUS and self.getDeviceState() == DeviceStateEnum.OPERATING and updated.mobility == False:
            self.safetyLightsPublisher.publish(toSafetyLights(True, False, 2, 150, "#00A36C"))
            
        if updated.state == SystemStateEnum.AUTONOMOUS and self.getDeviceState() == DeviceStateEnum.OPERATING and updated.mobility == True:
            self.safetyLightsPublisher.publish(toSafetyLights(True, False, 2, 150, "#841617"))

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

        if self.backCount == -1 and (lookahead is not None and ((lookahead[1] - cur_pos[1]) ** 2 + (lookahead[0] - cur_pos[0]) ** 2) > 0.1):
            angle_diff = math.atan2(lookahead[1] - cur_pos[1], lookahead[0] - cur_pos[0])
            error = self.getAngleDifference(angle_diff, self.position.theta) / math.pi
            self.log("Error: " + str(error))
            forward_speed = self.config.getFloat(FORWARD_SPEED) * (1 - abs(error)) ** 8
            inputPacket.forward_velocity = forward_speed
            error_sign = 1 if error > 0 else -1
            inputPacket.angular_velocity = 0.0 if abs(error) < 0.05 else (0.2 * error_sign) if abs(error) < (0.15 * error_sign) else (error * 1.3)
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
