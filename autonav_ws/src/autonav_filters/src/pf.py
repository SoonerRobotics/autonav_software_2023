#!/usr/bin/env python3

from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
from autonav_libs import AutoNode, Device, DeviceStateEnum, SystemStateEnum

import rclpy
import math
import random
from enum import IntEnum
import scipy
import numpy as np
from scipy import stats

from matplotlib import pyplot as plt


class Register(IntEnum):
    FILTER_TYPE = 0,
    SHOW_PLOT = 1


class FilterType(IntEnum):
    DEAD_RECKONING = 0,
    PARTICLE_FILTER = 1


class Particle:
    def __init__(self, x=0, y=0, theta=0, weight=1):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


class ParticleFilterNode(AutoNode):
    def __init__(self):
        super().__init__(Device.PARTICLE_FILTER, "autonav_filters_pf")

        self.m_pf = ParticleFilter(self)
        self.m_reckoning = DeadReckoningFilter(self)

        self.m_firstGps = None
        self.m_collecting = True

    def setup(self):
        self.config.writeInt(Register.FILTER_TYPE, FilterType.DEAD_RECKONING)
        self.config.writeBool(Register.SHOW_PLOT, True)
        self.setDeviceState(DeviceStateEnum.READY)
        self.setDeviceState(DeviceStateEnum.OPERATING)

        self.m_gpsSubscriber = self.create_subscription(GPSFeedback, "/autonav/gps", self.onGPSReceived, 20)
        self.m_motorFeedbackSubscriber = self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.onMotorFeedbackReceived, 20)
        self.m_posePublisher = self.create_publisher(Position, "/autonav/position", 20)

    def onGPSReceived(self, msg: GPSFeedback):
        if msg.gps_fix == 0 and msg.is_locked == False:
            return

        filterType = self.config.readInt(Register.FILTER_TYPE)
        if (self.m_firstGps is None or self.m_collecting) and filterType:
            if self.m_firstGps is None:
                self.m_firstGps = (msg.latitude, msg.longitude)
            else:
                self.m_firstGps = (0.8 * self.m_firstGps[0] + 0.2 * msg.latitude, 0.8 * self.m_firstGps[1] + 0.2 * msg.longitude)
            self.m_pf.setFirstGPS(self.m_firstGps[0], self.m_firstGps[1])
            return

        if filterType == FilterType.PARTICLE_FILTER:
            self.m_pf.updateGPS(msg)
        elif filterType == FilterType.DEAD_RECKONING:
            self.m_reckoning.updateGPS(msg)

    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        filterType = self.config.readInt(Register.FILTER_TYPE)
        if filterType == FilterType.PARTICLE_FILTER:
            self.m_pf.updateMotors(msg)
        elif filterType == FilterType.DEAD_RECKONING:
            self.m_reckoning.updateMotors(msg)

    def onSystemStateUpdated(self):
        if self.getSystemState() == SystemStateEnum.AUTONOMOUS:
            self.m_reckoning.reset()
        
        if self.getSystemState() == SystemStateEnum.AUTONOMOUS and self.m_collecting:
            self.m_collecting = False
            return

        if self.getSystemState() != SystemStateEnum.AUTONOMOUS:
            self.m_firstGps = None
            self.m_collecting = True
            self.m_pf.resetParticles()


class DeadReckoningFilter:
    def __init__(self, node: ParticleFilterNode):
        self.m_xSum = 0
        self.m_ySum = 0
        self.m_thetaSum = 0
        self.m_lastLat = None
        self.m_lastLong = None
        self.m_node = node
        self.m_firstGPS = None
        self.recordedPoints = []
        
    def rescalePlot(self):
        # Every time we add a new point, we rescale the plot to fit the new point
        plt.ylim(min(self.recordedPoints, key=lambda x: x[0])[0] - 1, max(self.recordedPoints, key=lambda x: x[0])[0] + 1)
        plt.xlim(min(self.recordedPoints, key=lambda x: x[1])[1] - 1, max(self.recordedPoints, key=lambda x: x[1])[1] + 1)
        
    def reset(self):
        self.m_xSum = 0
        self.m_ySum = 0
        self.m_thetaSum = 0
        self.m_lastLat = None
        self.m_lastLong = None
        self.m_firstGPS = None
        self.recordedPoints = []

    def updateMotors(self, feedback: MotorFeedback):
        self.m_xSum = self.m_xSum + feedback.delta_x * math.cos(self.m_thetaSum) + feedback.delta_y * math.sin(self.m_thetaSum)
        self.m_ySum = self.m_ySum + feedback.delta_x * math.sin(self.m_thetaSum) + feedback.delta_y * math.cos(self.m_thetaSum)
        # self.m_xSum += feedback.delta_x
        # self.m_ySum += feedback.delta_y
        self.m_thetaSum += feedback.delta_theta
        self.estimate()

    def updateGPS(self, msg: GPSFeedback):
        if msg.gps_fix <= 0 and msg.is_locked == False:
            return
        
        if self.m_firstGPS is None:
            self.m_firstGPS = (msg.latitude, msg.longitude)
        
        # latX = (msg.latitude - self.m_firstGPS[0]) * 111086.2
        # latY = (self.m_firstGPS[1] - msg.longitude) * 81978.2
        # dist = math.sqrt((latX - self.m_xSum) ** 2 + (latY - self.m_ySum) ** 2)
        
        # if dist > 5.0:
            # self.m_xSum = latX
            # self.m_ySum = latY

        self.m_lastLat = msg.latitude
        self.m_lastLong = msg.longitude
        self.estimate()

    def estimate(self):
        msg = Position()
        offset = 1
        if self.m_node.m_isSimulator:
            offset = 10
        msg.x = self.m_xSum / offset
        msg.y = self.m_ySum / offset
        msg.theta = self.m_thetaSum
        if self.m_lastLat is not None and self.m_lastLong is not None:
            msg.latitude = self.m_lastLat
            msg.longitude = self.m_lastLong

        self.m_node.m_posePublisher.publish(msg)

class ParticleFilter:
    def __init__(self, node):
        self.m_node = node
        self.m_numParticles = 500
        self.m_gpsNoise = [0.4]
        self.m_odomNoise = [0.05, 0.05, 0.1]

        self.resetParticles()

        self.m_firstGps = None
        plt.ion()

    def resetParticles(self):
        self.m_particles = [Particle(
            0, 0, i / self.m_numParticles * 2 * math.pi) for i in range(self.m_numParticles)]

    def gpsToXY(self, lat, long):
        x = (lat - self.m_firstGps[0]) * 111086.2
        y = (self.m_firstGps[1] - long) * 81978.2
        return (x, y)

    def setFirstGPS(self, lat, lon):
        self.m_firstGps = (lat, lon)

    def updateMotors(self, feedback: MotorFeedback):
        for particle in self.m_particles:
            particle.x += feedback.delta_x
            particle.y += feedback.delta_y
            particle.theta += feedback.delta_theta % (2 * math.pi)

    def updateGPS(self, msg: GPSFeedback):
        if self.m_firstGps is None:
            return

        gps_x = (msg.latitude - self.m_firstGps[0]) * 111086.2
        gps_y = (self.m_firstGps[1] - msg.longitude) * 81978.2

        for particle in self.m_particles:
            dist = math.sqrt((gps_x - particle.x) ** 2 +
                             (gps_y - particle.y) ** 2)
            particle.weight = scipy.stats.norm.pdf(dist, 0, self.m_gpsNoise[0])

        self.resample()
        self.visualize_particles()

    def resample(self):
        weight_sum = sum([particle.weight for particle in self.m_particles])
        if weight_sum == 0:
            self.resetParticles()
            return

        new_particles = random.choices(self.m_particles, weights=[
                                       particle.weight / weight_sum for particle in self.m_particles], k=self.m_numParticles)
        self.m_particles = []

        for particle in new_particles:
            new_particle = Particle(particle.x, particle.y, particle.theta)
            new_particle.x += np.random.normal(0, self.m_odomNoise[0])
            new_particle.y += np.random.normal(0, self.m_odomNoise[1])
            new_particle.theta = np.random.normal(
                particle.theta, self.m_odomNoise[2]) % (2 * math.pi)
            self.m_particles.append(new_particle)

    def estimate(self):
        pass

    def visualize_particles(self):
        if not self.m_node.config.readBool(Register.SHOW_PLOT) and plt.fignum_exists(1):
            plt.close(1)
            return

        if not self.m_node.config.readBool(Register.SHOW_PLOT):
            return

        plt.figure(1)
        plt.clf()

        # Draw all particles
        for point in self.m_particles:
            plt.quiver(point.x, point.y, math.cos(point.theta), math.sin(
                point.theta), color='b', scale=10 / point.weight, scale_units='width')

        plt.draw()
        plt.pause(0.00000001)


def main():
    rclpy.init()
    rclpy.spin(ParticleFilterNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
