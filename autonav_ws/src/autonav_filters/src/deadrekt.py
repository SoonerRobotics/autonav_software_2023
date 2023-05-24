from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
from scr_core.state import SystemMode
import math

class DeadReckoningFilter:
    def __init__(self):
        self.reset()
        
    def reset(self, heading: float = 0.0):
        self.xSum = 0.0
        self.ySum = 0.0
        self.thetaSum = heading
        self.latitude = 0.0
        self.longitude = 0.0

    def feedback(self, feedback: MotorFeedback):
        self.xSum = self.xSum + feedback.delta_x * math.cos(self.thetaSum) + feedback.delta_y * math.sin(self.thetaSum)
        self.ySum = self.ySum + feedback.delta_x * math.sin(self.thetaSum) + feedback.delta_y * math.cos(self.thetaSum)
        self.thetaSum += feedback.delta_theta
        
        return [self.xSum, self.ySum, self.thetaSum]

    def gps(self, feedback: GPSFeedback):
        self.latitude = feedback.latitude
        self.longitude = feedback.longitude