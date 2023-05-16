from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
from scr_core.state import SystemMode
import math

class DeadReckoningFilter:
    def __init__(self, node):
        self.node = node
        self.reset()
        
    def reset(self):
        self.xSum = 0
        self.ySum = 0
        self.thetaSum = 0.0
        self.lastLat = None
        self.lastLong = None

    def updateMotors(self, feedback: MotorFeedback):
        self.xSum = self.xSum + feedback.delta_x * math.cos(self.thetaSum) + feedback.delta_y * math.sin(self.thetaSum)
        self.ySum = self.ySum + feedback.delta_x * math.sin(self.thetaSum) + feedback.delta_y * math.cos(self.thetaSum)
        self.thetaSum += feedback.delta_theta
        self.broadcastEstimate()

    def updateGPS(self, gps: GPSFeedback):
        self.lastLat = gps.latitude
        self.lastLong = gps.longitude

    def broadcastEstimate(self):
        msg = Position()
        offset = 1
        if self.node.getSystemState().mode == SystemMode.SIMULATION:
            offset = 9.84251968503937
        msg.x = self.xSum / offset
        msg.y = self.ySum / offset
        msg.theta = self.thetaSum
        if self.lastLat is not None and self.lastLong is not None:
            msg.latitude = self.lastLat
            msg.longitude = self.lastLong

        self.node.positionPublisher.publish(msg)