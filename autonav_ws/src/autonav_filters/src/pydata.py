import particlefilter
import math
from autonav_msgs.msg import MotorFeedback, GPSFeedback, Position

def round_values_to_5_decimal_places(lst_of_lsts):
    rounded_lists = []
    for lst in lst_of_lsts:
        rounded_values = [round(val, 5) for val in lst]
        rounded_lists.append(rounded_values)
    return rounded_lists

def replace_brackets(string):
    replaced_string = string.replace("[", "{").replace("]", "}")
    return replaced_string

py_pf = particlefilter.ParticleFilter(111086.2, 81978.2)

py_pf.init_particles()

rounded_data = round_values_to_5_decimal_places(py_pf.get_particles_data())

data_string = replace_brackets(str(rounded_data))

#print(data_string)

delta_x_data = [0.0035,0.0171,0.0492]
delta_y_data = [0,0.0001,0.0006]
delta_theta_data = [0.003,0.0172,0.0154]

# get the data for first feedback
feedback_1 = MotorFeedback()

feedback_1.delta_x = float(delta_x_data[0])
feedback_1.delta_y = float(delta_y_data[0])
feedback_1.delta_theta = float(delta_theta_data[0])

avg_1 = py_pf.feedback(feedback_1)

print(f"avg_1 = {avg_1}")

data_1 = round_values_to_5_decimal_places(py_pf.get_particles_data())

string_data_1 = replace_brackets(str(data_1))

#print(string_data_1)

# get the data for second feedback

feedback_2 = MotorFeedback()
feedback_2.delta_x = float(delta_x_data[1])
feedback_2.delta_y = float(delta_y_data[1])
feedback_2.delta_theta = float(delta_theta_data[1])

avg_2 = py_pf.feedback(feedback_2)

print(f"avg_2 = {avg_2}")

# get the data for third feedback

feedback_3 = MotorFeedback()

feedback_3.delta_x = float(delta_x_data[2])
feedback_3.delta_y = float(delta_y_data[2])
feedback_3.delta_theta = float(delta_theta_data[2])

avg_3 = py_pf.feedback(feedback_3)

print(f"avg_3 = {avg_3}")

