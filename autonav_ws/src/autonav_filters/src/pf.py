#!/usr/bin/env python3

from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
from autonav_libs import AutoNode, Device, DeviceStateEnum, SystemStateEnum

import rclpy
import math
import random
import numpy as np
from enum import IntEnum

import numpy as np
from numpy.random import randn, random, uniform
from matplotlib import pyplot as plt
import scipy.stats

class Register(IntEnum):
    SHOW_PLOT = 0

class Particle:
    def __init__(self, x = 0, y = 0, theta = 0, weight = 1):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


class ParticleFilter:
    def __init__(self, node):
        self.m_node = node
        self.m_numParticles = 500
        self.m_gpsNoise = [0.4]
        self.m_odomNoise = [0.05, 0.05, 0.1]
        
        self.resetParticles()
        
        self.m_firstGps = None
        
    def resetParticles(self):
        self.m_particles = [Particle(0, 0, i / self.m_numParticles * 2 * math.pi) for i in range(self.m_numParticles)]
            
    def gpsToXY(self, lat, long):
        x = (lat - self.m_firstGps[0]) * 111086.2
        y = (self.m_firstGps[1] - long) * 81978.2
        return (x, y)
    
    def setFirstGPS(self, lat, lon):
        self.m_firstGps = (lat, lon)
            
    def updateMotors(self, feedback: MotorFeedback):
        for particle in self.m_particles:
            particle.x += feedback.delta_x * 1.2 * math.cos(particle.theta) + feedback.delta_y * math.sin(particle.theta)
            particle.y += feedback.delta_x * 1.2 * math.sin(particle.theta) - feedback.delta_y * math.cos(particle.theta)
            particle.theta += feedback.delta_theta
            particle.theta %= 2 * math.pi
    
    def updateGPS(self, msg: GPSFeedback):
        if self.m_firstGps is None:
            return
        
        gps_x = (msg.latitude - self.m_firstGps[0]) * 111086.2
        gps_y = (self.m_firstGps[1] - msg.longitude) * 81978.2
        
        for particle in self.m_particles:
            dist = math.sqrt((gps_x - particle.x) ** 2 + (gps_y - particle.y) ** 2)
            particle.weight = scipy.stats.norm.pdf(dist, 0, self.m_gpsNoise[0])
            
        self.resample()
        self.visualize_particles()

    def resample(self):
        weight_sum = sum([particle.weight for particle in self.m_particles])
        if weight_sum == 0:
            # Something is not right :(
            self.resetParticles()
            return
        
        new_particles = random.choices(self.m_particles, weights=[particle.weight / weight_sum for particle in self.m_particles], k=self.m_numParticles)
        self.m_particles = []
        
        for particle in new_particles:
            new_particle = Particle(particle.x, particle.y, particle.theta)
            new_particle.x += randn() * self.m_odomNoise[0]
            new_particle.y += randn() * self.m_odomNoise[1]
            new_particle.theta += randn() * self.m_odomNoise[2]
            new_particle.theta %= 2 * math.pi
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
            plt.quiver(point.x, point.y, math.cos(point.theta), math.sin(point.theta), color='b', scale=10 / point.weight, scale_units='width')

        plt.draw()
        plt.pause(0.00000001)


class ParticleFilterNode(AutoNode):
    def __init__(self):
        super().__init__(Device.PARTICLE_FILTER, "autonav_filters_pf")

        self.m_firstGps = None
        self.m_collecting = True

    def setup(self):
        self.config.writeBool(Register.SHOW_PLOT, True)
        self.setDeviceState(DeviceStateEnum.READY)
        self.m_pf = ParticleFilter(self)
        
        self.m_gpsSubscriber = self.create_subscription(GPSFeedback, "/autonav/gps", self.onGPSReceived, 20)
        self.m_motorFeedbackSubscriber = self.create_subscription(MotorFeedback, "/autonav/MotorFeedback", self.onMotorFeedbackReceived, 20)
        self.m_posePublisher = self.create_publisher(Position, "/autonav/position", 20)

    def onGPSReceived(self, msg: GPSFeedback):
        if msg.gps_fix == 0 and msg.is_locked == False:
            return
        
        if self.m_firstGps is None or self.m_collecting:
            # if self.m_firstGps is None:
            self.m_firstGps = (msg.latitude, msg.longitude)
            # else:
                # self.m_firstGps = (0.8 * self.m_firstGps[0] + 0.2 * msg.latitude, 0.8 * self.m_firstGps[1] + 0.2 * msg.longitude)
            self.log(f"[Collecting] Setting GPS -> ({self.m_firstGps[0]}, {self.m_firstGps[1]})")
            self.m_pf.setFirstGPS(self.m_firstGps[0], self.m_firstGps[1])
            return
        
        self.m_pf.updateGPS(msg)

    def onMotorFeedbackReceived(self, msg: MotorFeedback):
        self.m_pf.updateMotors(msg)
        # avg_x, avg_y, avg_theta = self.m_pf.estimate()
        # output = Position()
        # print(avg_x, avg_y, avg_theta)
        # output.x = avg_x
        # output.y = avg_y
        # output.theta = avg_theta
        
        # if self.m_firstGps is not None:
            # output.latitude = self.m_firstGps[0] + avg_x / 111086.2
            # output.longitude = self.m_firstGps[1] - avg_y / 81978.2
        
        # self.m_posePublisher.publish(output)

        
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
