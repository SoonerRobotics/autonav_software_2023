from autonav_msgs.msg import MotorFeedback, GPSFeedback

import math
import random
import scipy
import numpy as np
from matplotlib import pyplot as plt

class Particle:
    def __init__(self, x=0, y=0, theta=0, weight=1):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

class ParticleFilter:
    def __init__(self, node):
        self.node = node
        self.numParticles = 500
        self.gpsNoise = [0.4]
        self.odomNoise = [0.05, 0.05, 0.1]

        self.resetParticles()
        self.firstGps = None

    def resetParticles(self):
        self.particles = [Particle(0, 0, i / self.numParticles * 2 * math.pi) for i in range(self.numParticles)]

    def gpsToXY(self, lat, long):
        x = (lat - self.firstGps[0]) * 111086.2
        y = (self.firstGps[1] - long) * 81978.2
        return (x, y)

    def setFirstGPS(self, lat, lon):
        self.firstGps = (lat, lon)

    def updateMotors(self, feedback: MotorFeedback):
        for particle in self.particles:
            particle.x += feedback.delta_x
            particle.y += feedback.delta_y
            particle.theta += feedback.delta_theta % (2 * math.pi)

    def updateGPS(self, msg: GPSFeedback):
        if self.firstGps is None:
            return

        gps_x = (msg.latitude - self.firstGps[0]) * 111086.2
        gps_y = (self.firstGps[1] - msg.longitude) * 81978.2

        for particle in self.particles:
            dist = math.sqrt((gps_x - particle.x) ** 2 + (gps_y - particle.y) ** 2)
            particle.weight = scipy.stats.norm.pdf(dist, 0, self.gpsNoise[0])

        self.resample()
        self.visualize_particles()

    def resample(self):
        weight_sum = sum([particle.weight for particle in self.particles])
        if weight_sum == 0:
            self.resetParticles()
            return

        new_particles = random.choices(self.particles, weights=[particle.weight / weight_sum for particle in self.particles], k=self.numParticles)
        self.particles = []

        for particle in new_particles:
            new_particle = Particle(particle.x, particle.y, particle.theta)
            new_particle.x += np.random.normal(0, self.odomNoise[0])
            new_particle.y += np.random.normal(0, self.odomNoise[1])
            new_particle.theta = np.random.normal(
                particle.theta, self.odomNoise[2]) % (2 * math.pi)
            self.particles.append(new_particle)

    def estimate(self):
        pass