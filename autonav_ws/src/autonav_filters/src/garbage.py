import math
import numpy as np
import particlefilter
import pandas
import time

from autonav_msgs.msg import MotorFeedback, Position, GPSFeedback

motor_feedback_data = pandas.read_csv("~/Downloads/igvc_data_2023/day3_p2/autonomous_2023-06-04_13-10-36/ENTRY_FEEDBACK.csv")

gps_feedback_data = pandas.read_csv("~/Downloads/igvc_data_2023/day3_p2/autonomous_2023-06-04_13-10-36/ENTRY_GPS.csv")

# motor

delta_xs = motor_feedback_data.iloc[:, 2].to_list()
delta_ys = motor_feedback_data.iloc[:, 3].to_list()
delta_thetas = motor_feedback_data.iloc[:, 4].to_list()

# gps
latitudes = gps_feedback_data.iloc[:, 2].to_list()
longitudes = gps_feedback_data.iloc[:, 3].to_list()
altitudes = gps_feedback_data.iloc[:, 4].to_list()
gps_fix = gps_feedback_data.iloc[:, 5].to_list()
is_locked = gps_feedback_data.iloc[:, 6].to_list()
satellites = gps_feedback_data.iloc[:, 7].to_list()

# assuming im reading csvs right lol

proper_len = len(latitudes)
#proper_len = 10
delta_xs = delta_xs[0:proper_len]
delta_ys = delta_ys[0:proper_len]
delta_thetas = delta_thetas[0:proper_len]
print(len(delta_xs))
# create feedback messages

motor_feedbacks = []

gps_feedbacks = []

for i in range(len(delta_xs)):
    motor_message = MotorFeedback()
    motor_message.delta_x = delta_xs[i]
    print(f"motor_message delta_x {motor_message.delta_x}")
    motor_message.delta_y = delta_ys[i]
    print(f"motor_message delta_y {motor_message.delta_y}")
    motor_message.delta_theta = delta_thetas[i]
    print(f"motor_message delta_theta {motor_message.delta_theta}\n")

    motor_feedbacks.append(motor_message)

    gps_message = GPSFeedback()
    gps_message.latitude = latitudes[i]
    gps_message.longitude = longitudes[i]
    gps_message.altitude = altitudes[i]
    gps_message.gps_fix = gps_fix[i]
    gps_message.is_locked = eval(is_locked[i])
    gps_message.satellites = satellites[i]

    gps_feedbacks.append(gps_message)

print(len(motor_feedbacks))
print(len(gps_feedbacks))

py_particle_filter = particlefilter.ParticleFilter(111086.2, 81978.2)
py_particle_filter.init_particles()

for i in range(len(motor_feedbacks)):
    position_vector = py_particle_filter.feedback(motor_feedbacks[i])
    gps_vector = py_particle_filter.gps(gps_feedbacks[i])

print(position_vector)
print(gps_vector)

end_result = [18.49585229987204, 18.936964199952953]
