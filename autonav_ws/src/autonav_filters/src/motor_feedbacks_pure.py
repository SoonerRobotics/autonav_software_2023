import math
import numpy as np
import particlefilter
import pandas
import time

from autonav_msgs.msg import MotorFeedback, Position, GPSFeedback

motor_feedback_data = pandas.read_csv("~/Downloads/igvc_data_2023/day3_p2/autonomous_2023-06-04_13-10-36/ENTRY_FEEDBACK.csv")

# motor

delta_xs = motor_feedback_data.iloc[:1402, 2].to_list()
delta_ys = motor_feedback_data.iloc[:1402, 3].to_list()
delta_thetas = motor_feedback_data.iloc[:1402, 4].to_list()

print(len(delta_xs))

motor_feedbacks = []

for i in range(len(delta_xs)):
    motor_message = MotorFeedback()
    motor_message.delta_x = delta_xs[i]
    motor_message.delta_y = delta_ys[i]
    motor_message.delta_theta = delta_thetas[i]

    motor_feedbacks.append(motor_message)

py_particle_filter = particlefilter.ParticleFilter(111086.2, 81978.2)
py_particle_filter.init_particles()

for i in range(len(motor_feedbacks)):
    position_vector = py_particle_filter.feedback(motor_feedbacks[i])

print(position_vector)

end_result = [1.4589810840940724e-15, -1.1226575225009583e-14, 3.4728605908690113]