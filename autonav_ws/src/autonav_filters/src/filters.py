#!/usr/bin/env python3

from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
from scr_msgs.msg import SystemState
from scr_core.node import Node
from scr_core.state import DeviceStateEnum, SystemStateEnum
from particlefilter import ParticleFilter
from deadrekt import DeadReckoningFilter
from enum import IntEnum

import rclpy

class FilterType(IntEnum):
    DEAD_RECKONING = 0,
    PARTICLE_FILTER = 1

class FiltersNode(Node):
    def __init__(self):
        super().__init__("autonav_filters")

        self.pf = ParticleFilter()
        self.reckoning = DeadReckoningFilter(self)

    def configure(self):
        self.config.setInt(0, FilterType.DEAD_RECKONING)

        self.create_subscription(GPSFeedback, "/autonav/gps", self.onGPSReceived, 20)
        self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.onMotorFeedbackReceived, 20)
        self.positionPublisher = self.create_publisher(Position, "/autonav/position", 20)
        self.collecting = True
        self.first_gps = None

        self.setDeviceState(DeviceStateEnum.OPERATING)

    def onReset(self):
        self.pf.init_particles()
        self.reckoning.reset()

    def transition(self, old: SystemState, updated: SystemState):
        if old.state != SystemStateEnum.AUTONOMOUS and updated.state == SystemStateEnum.AUTONOMOUS:
            self.reckoning.reset()
            
        if old.state == SystemStateEnum.AUTONOMOUS and updated.state != SystemStateEnum.AUTONOMOUS:
            self.pf.init_particles()
          
        if old.state != SystemStateEnum.MANUAL and updated.state == SystemStateEnum.MANUAL:
            self.onReset()

    def onGPSReceived(self, msg: GPSFeedback):
        if msg.gps_fix == 0 and msg.is_locked == False:
            return
        
        if self.first_gps is None:
            self.first_gps = msg

        filterType = self.config.getInt(0)
        if filterType == FilterType.PARTICLE_FILTER:
            self.pf.gps(msg)
        elif filterType == FilterType.DEAD_RECKONING:
            self.reckoning.updateGPS(msg)

    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        filterType = self.config.getInt(0)
        if filterType == FilterType.PARTICLE_FILTER:
            averages = self.pf.feedback(msg)
            position = Position()
            position.x = averages[0]
            position.y = averages[1]
            position.theta = averages[2]
            
            if self.first_gps is not None:
                gps_x = self.first_gps.latitude + averages[0] / 111086.2
                gps_y = self.first_gps.longitude - averages[1] / 81978.2
                position.latitude = gps_x
                position.longitude = gps_y
            
            self.positionPublisher.publish(position)
        
        if filterType == FilterType.DEAD_RECKONING:
            self.reckoning.updateMotors(msg)


def main():
    rclpy.init()
    rclpy.spin(FiltersNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
