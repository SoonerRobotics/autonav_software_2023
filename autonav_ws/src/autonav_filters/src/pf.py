#!/usr/bin/env python3

from autonav_msgs.msg import MotorFeedback, GPSFeedback
from autonav_libs import AutoNode, Device, DeviceStateEnum, SystemStateEnum
from geometry_msgs.msg import Pose, Point

import rclpy
import math
import random
import numpy as np

from matplotlib import pyplot as plt

class Particle:
    def __init__(self, x=0, y=0, weight=0, theta=0):
        self.x = x
        self.y = y
        self.weight = weight
        self.theta = theta


class ParticleFilter:
    def __init__(self):
        self.m_numParticles = 500
        self.m_gpsNoise = [0.4]
        self.m_odomNoise = [0.05, 0.05, 0.1]
        
        self.resetParticles()
        
        self.m_firstGps = None
        self.m_lastGps = None
        self.m_lastPose = (0, 0, 0)
        self.gps_points = []

    def resetParticles(self):
        self.m_particles = []
        for i in range(self.m_numParticles):
            particle = Particle(
                0, 0, i / self.m_numParticles * 2 * math.pi, 0
            )
            self.m_particles.append(particle)
            
    def gpsToXY(self, gps: GPSFeedback):
        x = (gps.latitude - self.m_firstGps[0]) * 111086.2
        y = (self.m_firstGps[1] - gps.longitude) * 81978.2
        return (x, y)
    
    def setFirstGPS(self, gps: GPSFeedback):
        self.m_firstGps = (gps.latitude, gps.longitude)
            
    def updateMotors(self, feedback: MotorFeedback):
        sum_x = 0
        sum_y = 0
        sum_theta_x = 0
        sum_theta_y = 0
        sum_weight = 0
        
        for particle in self.m_particles:
            particle.x += feedback.delta_x * 1.2 * math.cos(particle.theta) + feedback.delta_y * math.sin(particle.theta)
            particle.y += feedback.delta_x * 1.2 * math.sin(particle.theta) + feedback.delta_y * math.cos(particle.theta)
            particle.theta += feedback.delta_theta % (2 * math.pi)
            weight = math.pow(particle.weight, 2)
            
            sum_x += particle.x * weight
            sum_y += particle.y * weight
            sum_theta_x += math.cos(particle.theta) * weight
            sum_theta_y += math.sin(particle.theta) * weight
            sum_weight += weight
            
        if sum_weight < 0.00001:
            sum_weight = 0.00001
            
        avg_x = sum_x / sum_weight
        avg_y = sum_y / sum_weight
        avg_theta = math.atan2(sum_theta_y, sum_theta_x) % (2 * math.pi)
        return (avg_x, avg_y, avg_theta)
    
    def updateGPS(self, msg: GPSFeedback):
        if self.m_firstGps is None:
            self.m_firstGps = (msg.latitude, msg.longitude)

        xy = self.gpsToXY(msg)
        if self.m_lastGps is None:
            self.m_lastGps = xy
        
        difx = xy[0] - self.m_lastGps[0]
        dify = xy[1] - self.m_lastGps[1]
        diftheta = math.atan2(dify, difx) % (2 * math.pi)
        
        for particle in self.m_particles:
            distance = np.sqrt(math.pow(particle.x - xy[0], 2) + math.pow(particle.y - xy[1], 2))
            particle.weight = math.exp(-distance / (2 * math.pow(self.m_gpsNoise[0], 2)))
        
        self.shiftParticles()
        self.m_lastGps = xy
        self.gps_points.append(xy)
        return xy

    def shiftParticles(self):
        weights = [particle.weight for particle in self.m_particles]
        total_weight = sum(weights)
        if total_weight < 0.00001:
            total_weight = 0.00001
        weights = [weight / total_weight for weight in weights]
        
        new_particles = random.choices(self.m_particles, weights, k=self.m_numParticles)
        self.m_particles = []
        
        for particle in new_particles:
            randxy = (np.random.normal(0, self.m_odomNoise[0]), np.random.normal(0, self.m_odomNoise[1]))
            x = particle.x + randxy[0] * math.cos(particle.theta) + randxy[1] * math.sin(particle.theta)
            y = particle.y + randxy[0] * math.sin(particle.theta) + randxy[1] * math.cos(particle.theta)
            theta = np.random.normal(particle.theta, self.m_odomNoise[2]) % (2 * math.pi)
            self.m_particles.append(Particle(x, y, particle.weight, theta))


class ParticleFilterNode(AutoNode):
    def __init__(self):
        super().__init__(Device.PARTICLE_FILTER, "autonav_filters_pf")

        self.m_gpsSubscriber = self.create_subscription(GPSFeedback, "/autonav/gps", self.onGPSReceived, 20)
        self.m_motorFeedbackSubscriber = self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.onMotorFeedbackReceived, 20)
        self.m_posePublisher = self.create_publisher(Pose, "/autonav/pose", 20)

        self.m_firstGps = None
        self.m_collecting = True
        self.m_pf = ParticleFilter()

    def setup(self):
        self.setDeviceState(DeviceStateEnum.READY)

    def onGPSReceived(self, msg: GPSFeedback):
        if msg.gps_fix == 0 and msg.is_locked == False:
            return
        
        if self.m_firstGps is None or self.m_collecting:
            self.m_firstGps = (msg.latitude, msg.longitude)
            self.log(f"Setting first GPS -> ({self.m_firstGps[0]}, {self.m_firstGps[1]})")
            self.m_pf.setFirstGPS(msg)
            return
        
        self.log(f"GPS -> ({msg.latitude}, {msg.longitude})")
        self.m_pf.updateGPS(msg)

    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        avg_x, avg_y, avg_theta = self.m_pf.updateMotors(msg)
        output = Pose()
        point = Point()
        point.x = avg_x
        point.y = avg_y
        point.z = avg_theta
        
        if self.m_firstGps is not None:
            point.x = self.m_firstGps[0] + avg_x / 111086.2
            point.y = self.m_firstGps[1] - avg_y / 81978.2
            
        if self.m_firstGps is not None:
            lat = self.m_firstGps[0] + avg_x / 111086.2
            lon = self.m_firstGps[1] - avg_y / 81978.2
            # self.log(f"({lat}, {lon})")
        
        output.position = point
        self.m_posePublisher.publish(output)

        
    def onSystemStateUpdated(self):
        if self.getSystemState() == SystemStateEnum.AUTONOMOUS and self.m_collecting:
            self.m_collecting = False
            self.log("Starting particle filter")
            return
        
        if self.getSystemState() != SystemStateEnum.AUTONOMOUS:
            self.m_firstGps = None
            self.m_collecting = True
            self.log("Resetting particle filter")
            self.m_pf.resetParticles()


def main():
    rclpy.init()

    plt.ion()
    
    rclpy.spin(ParticleFilterNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
