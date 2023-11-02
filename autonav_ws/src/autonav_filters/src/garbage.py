import math
import particlefilter
import pandas

from autonav_msgs.msg import MotorFeedback, GPSFeedback

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



