import particlefilter
from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position

py_pf = particlefilter.ParticleFilter(111086.2, 81978.2)

delta_x_data = [0.0035,0.0171,0.0492]
delta_y_data = [0,0.0001,0.0006,0.0004]
delta_theta_data = [0.003,0.0172,0.0154]
position_data = []

message = MotorFeedback()
message.delta_x = float(delta_x_data[0])
message.delta_y = float(delta_y_data[0])
message.delta_theta = float(delta_theta_data[0])
avg_vector_0 = py_pf.feedback(message)
for particle in py_pf.particles:
    position_data.append([particle.x, particle.y, particle.theta])

print(position_data)
print(len(position_data))
print(avg_vector_0)

position_data = []

message.delta_x = float(delta_x_data[1])
message.delta_y = float(delta_y_data[1])
message.delta_theta = float(delta_theta_data[1])

avg_vector_1 = py_pf.feedback(message)
for particle in py_pf.particles:
    position_data.append([particle.x, particle.y, particle.theta])

#print(position_data)
print(len(position_data))
print(avg_vector_1)

position_data = []

message.delta_x = float(delta_x_data[2])
message.delta_y = float(delta_y_data[2])
message.delta_theta = float(delta_theta_data[2])

avg_vector_2 = py_pf.feedback(message)
for particle in py_pf.particles:
    position_data.append([particle.x, particle.y, particle.theta])

#print(position_data)
print(avg_vector_2)

