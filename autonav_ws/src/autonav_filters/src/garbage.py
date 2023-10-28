import particlefilter
import math
from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position

py_pf = particlefilter.ParticleFilter(111086.2, 81978.2)

py_pf.init_particles()

message = MotorFeedback()

delta_x_data = [0.0035,0.0171,0.0492]
delta_y_data = [0,0.0001,0.0006]
delta_theta_data = [0.003,0.0172,0.0154]

message.delta_x = float(delta_x_data[0])
#message.delta_x = 300.0
message.delta_y = float(delta_y_data[0])
#message.delta_y = 500.0
message.delta_theta = float(delta_theta_data[0])

avg_vector_1 = py_pf.feedback(message)

print(avg_vector_1)

message_2 = MotorFeedback()
message_2.delta_x = float(delta_x_data[1])
message_2.delta_y = float(delta_y_data[1])
message_2.delta_theta = float(delta_theta_data[1])

avg_vector_2 = py_pf.feedback(message_2)

print(avg_vector_2)

