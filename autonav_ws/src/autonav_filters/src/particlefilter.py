from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position
import numpy as np
import math
import random

class Particle:
    def __init__(self, x = 0, y = 0, theta = 0, weight = 1) -> None:
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

class ParticleFilter:
    def __init__(self, latitudeLength, longitudeLength) -> None:
        self.num_particles = 10
        self.gps_noise = [0.45]
        self.odom_noise = [0.05, 0.05, 0.1]
        self.init_particles()
        self.first_gps = None
        
        self.latitudeLength = latitudeLength
        self.longitudeLength = longitudeLength

    def init_particles(self, seedHeading: float = 0.0, useSeedHeading: bool = False):
        if useSeedHeading:
            #random.seed(30)
            self.particles = [Particle(0, 0, seedHeading + np.random.normal(0, 0.1)) for i in range(self.num_particles)]
        else:
            self.particles = [Particle(0, 0, i / self.num_particles * 2 * math.pi) for i in range(self.num_particles)]

    def feedback(self, feedback: MotorFeedback) -> list[float]:
        sum_x = 0
        sum_y = 0
        sum_theta_x = 0
        sum_theta_y = 0
        sum_weight = 0
        i = 0
        
        for particle in self.particles:
            print(f"particle data: {particle.x}, {particle.y}, {particle.theta}, {particle.weight}\n")
            particle.x += feedback.delta_x * 1.2 * math.cos(particle.theta) + feedback.delta_y * math.sin(particle.theta)
            particle.y += feedback.delta_x * 1.2 * math.sin(particle.theta) + feedback.delta_y * math.cos(particle.theta)
            particle.theta += feedback.delta_theta
            particle.theta = particle.theta % (2 * math.pi)
            print(f"particle data after: {particle.x}, {particle.y}, {particle.theta}, {particle.weight}\n")
            weight = particle.weight ** 2
            sum_x += particle.x * weight
            sum_y += particle.y * weight
            sum_theta_x += math.cos(particle.theta) * weight
            sum_theta_y += math.sin(particle.theta) * weight
            sum_weight += weight

            print(f"summation data: {sum_x}, {sum_y}, {sum_theta_x}, {sum_theta_y}, {sum_weight}\n")
            print(f"iteration number: {i}\n")
            i += 1
            
        if sum_weight < 0.000001:
            sum_weight = 0.000001
            
        avg_x = sum_x / sum_weight
        avg_y = sum_y / sum_weight
        avg_theta = math.atan2(sum_theta_y / sum_weight, sum_theta_x / sum_weight) % (2 * math.pi)

        print(f"average data: {avg_x}, {avg_y}, {avg_theta}\n")
        for particle in self.particles:
            print(f"new particles in feedback: {particle.x}, {particle.y}, {particle.theta}, {particle.weight}")

        print(f"\n====== END FEEDBACK ======\n")
        return [avg_x, avg_y, avg_theta]
    
    def gps(self, gps: GPSFeedback) -> list[float]:
        if self.first_gps is None:
            self.first_gps = gps
            
        gps_x = (gps.latitude - self.first_gps.latitude) * self.latitudeLength
        gps_y = (self.first_gps.longitude - gps.longitude) * self.longitudeLength
    
        print(f"gps_x, gps_y: {gps_x}, {gps_y}")

        for particle in self.particles:
            print(f"particle_x, particle_y: {particle.x}, {particle.y}")
            dist_sqrt = np.sqrt((particle.x - gps_x) ** 2 + (particle.y - gps_y) ** 2)
            print(f"dist_sqrt {dist_sqrt}")
            particle.weight = math.exp(-dist_sqrt / (2 * self.gps_noise[0] ** 2))
            print(f"particle weight after reassignment {particle.weight}")
            
        self.resample()
        #print(f"gps_vector in particle_filter header: x: {gps_x}, y: {gps_y}")
        gps_log_file = open("py_gps_log.txt", "a")

        individual_particles_string = ""
        for particle in self.particles:
            individual_particles_string = individual_particles_string + f"{particle.x}, {particle.y}, "
            #print(f"{particle.x}, {particle.y}")

        #print(f"individual particle string is {individual_particles_string}")
        individual_particles_string = individual_particles_string + f"{gps_x}, {gps_y}\n"
        gps_log_file.write(individual_particles_string)
        gps_log_file.close()

        print(f"\n====== END GPS ======\n")
        return [gps_x, gps_y]
    
    def resample(self) -> None:
        weights = [particle.weight for particle in self.particles]
        for weight in weights:
            print(f"weight: {weight}")
        

        weights_sum = sum(weights)
        print(f"weight sum: {weights_sum}")
        if weights_sum <= 0.00001:
            weights_sum = 0.00001
        weights = [weight / weights_sum for weight in weights]
        for weight in weights:
            print(f"weight after sum: {weight}")

        #random.seed(30)
        new_particles = random.choices(self.particles, weights, k = self.num_particles)

        for particle in new_particles:
            print(f"new particles: {particle.x}, {particle.y}, {particle.theta}, {particle.weight}")

        self.particles = []
        
        for particle in new_particles:
            #np.random.seed(30)
            rand_x = np.random.normal(0, self.odom_noise[0])
            #np.random.seed(30)
            rand_y = np.random.normal(0, self.odom_noise[1])

            print(f"rand_x, rand_y: {rand_x}, {rand_y}")
            x = particle.x + rand_x * math.cos(particle.theta) + rand_y * math.sin(particle.theta)
            y = particle.y + rand_x * math.sin(particle.theta) + rand_y * math.cos(particle.theta)
            print(f"x, y: {x}, {y}")
            #np.random.seed(30)
            theta = np.random.normal(particle.theta, self.odom_noise[2]) % (2 * math.pi)
            self.particles.append(Particle(x, y, theta, particle.weight))
        
        for particle in self.particles:
            print(f"new particles after resample: {particle.x}, {particle.y}, {particle.theta}, {particle.weight}")