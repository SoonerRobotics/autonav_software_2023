#!/usr/bin/env python3

from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
from autonav_libs import AutoNode, DeviceStateEnum, SystemStateEnum
from .particlefilter import ParticleFilter
from .deadrekt import DeadReckoningFilter
from enum import IntEnum

import rclpy

class FilterType(IntEnum):
    DEAD_RECKONING = 0,
    PARTICLE_FILTER = 1

class ParticleFilterNode(AutoNode):
    def __init__(self):
        super().__init__("autonav_filters")

        self.pf = ParticleFilter(self)
        self.reckoning = DeadReckoningFilter(self)

    def setup(self):
        self.config.writeInt(0, FilterType.DEAD_RECKONING)
        self.config.writeBool(1, True)

        self.create_subscription(GPSFeedback, "/autonav/gps", self.onGPSReceived, 20)
        self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.onMotorFeedbackReceived, 20)
        self.positionPublisher = self.create_publisher(Position, "/autonav/position", 20)

        self.setDeviceState(DeviceStateEnum.READY)
        self.setDeviceState(DeviceStateEnum.OPERATING)

    def onGPSReceived(self, msg: GPSFeedback):
        if msg.gps_fix == 0 and msg.is_locked == False:
            return

        filterType = self.config.readInt(0)
        if filterType == FilterType.PARTICLE_FILTER:
            self.pf.updateGPS(msg)
        elif filterType == FilterType.DEAD_RECKONING:
            self.reckoning.updateGPS(msg)

    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        filterType = self.config.readInt(0)
        if filterType == FilterType.PARTICLE_FILTER:
            self.pf.updateMotors(msg)
        elif filterType == FilterType.DEAD_RECKONING:
            self.reckoning.updateMotors(msg)

    def onSystemStateUpdated(self):
        if self.getSystemState() == SystemStateEnum.AUTONOMOUS:
            self.reckoning.reset()

        if self.getSystemState() != SystemStateEnum.AUTONOMOUS:
            self.pf.resetParticles()



def main():
    rclpy.init()
    rclpy.spin(ParticleFilterNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
