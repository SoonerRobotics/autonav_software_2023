from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
import numpy as np
import random
import math

class Particle:
    def __init__(self, x = 0, y = 0, theta = 0, weight = 1):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

class ParticleFilter:
    def __init__(self, node, numParticles = 500, gps_noise = None, feedback_noise = None):
        self.node = node
        self.numParticles = numParticles
        self.gpsNoise = gps_noise if gps_noise is not None else [0.2]
        self.odomNoise = feedback_noise if feedback_noise is not None else [0.5, 0.5, 0.1]
        self.resetParticles()
        self.firstGps = None

    def resetParticles(self):
        self.particles = [Particle(0, 0, i / self.numParticles * 2 * math.pi) for i in range(self.numParticles)]

    def updateMotors(self, feedback: MotorFeedback):
        sum_x = 0
        sum_y = 0
        sum_theta_x = 0
        sum_theta_y = 0
        sum_weight = 0

        for particle in self.particles:
            particle.x += feedback.delta_x * 1.2 * math.cos(particle.theta) + feedback.delta_y * math.sin(particle.theta)
            particle.y += feedback.delta_x * 1.2 * math.sin(particle.theta) + feedback.delta_y * math.cos(particle.theta)
            particle.theta += feedback.delta_theta
            particle.theta = particle.theta % (2 * math.pi)

            weight = particle.weight ** 2

            sum_x += particle.x * weight
            sum_y += particle.y * weight
            sum_theta_x += math.cos(particle.theta) * weight
            sum_theta_y += math.sin(particle.theta) * weight
            sum_weight += weight

        if sum_weight  < 0.000001:
            sum_weight = 0.000001

        avg_x = sum_x / sum_weight
        avg_y = sum_y / sum_weight
        avg_theta = math.atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight) % (2 * math.pi)

        msg = Position()
        msg.x = avg_x / 10.0
        msg.y = avg_y / 10.0
        msg.theta = avg_theta
        if self.firstGps is not None:
            msg.latitude = self.firstGps[0] + (avg_x / 10.0) / 111086.2
            msg.longitude = self.firstGps[1] - (avg_y / 10.0) / 81978.2

        self.node.positionPublisher.publish(msg)

    def updateGPS(self, msg: GPSFeedback, collecting: bool):
        if collecting:
            self.firstGps = (msg.latitude, msg.longitude)
            return
    
        gps_x = (msg.latitude - self.firstGps[0]) * 111086.2
        gps_y = (self.firstGps[1] - msg.longitude) * 81978.2

        for particle in self.particles:
            dist_sqr = np.sqrt((particle.x - gps_x) ** 2 + (particle.y - gps_y) ** 2)
            particle.weight = math.exp(-dist_sqr / (2 * self.gpsNoise[0] ** 2))

        self.resample()
        return (gps_x, gps_y)
    
    def resample(self):
        weights = [particle.weight for particle in self.particles]
        weights_sum = sum(weights)
        if weights_sum < 0.00001:
            weights_sum = 0.00001
        weights = [weight / weights_sum for weight in weights]

        new_particles = random.choices(self.particles, weights, k=self.numParticles)
        self.particles = []

        for particle in new_particles:
            rand_x = np.random.normal(0, self.odomNoise[0])
            rand_y = np.random.normal(0, self.odomNoise[1])
            x = particle.x + rand_x * math.cos(particle.theta) + rand_y * math.sin(particle.theta)
            y = particle.y + rand_x * math.sin(particle.theta) + rand_y * math.cos(particle.theta)
            theta = np.random.normal(particle.theta, self.odomNoise[2]) % (2 * math.pi)
            self.particles.append(Particle(x, y, theta, particle.weight))                