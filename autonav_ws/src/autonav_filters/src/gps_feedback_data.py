import particlefilter
from autonav_msgs.msg import GPSFeedback, Position

py_pf = particlefilter.ParticleFilter(111086.2, 81978.2)

py_pf.init_particles()

gps_feedback_1 = GPSFeedback()

gps_feedback_1.latitude = 42.6681254
gps_feedback_1.longitude = -83.2188876
gps_feedback_1.altitude = 234.891
gps_feedback_1.gps_fix = 4
gps_feedback_1.is_locked = True
gps_feedback_1.satellites = 16

gps_vector_1 = py_pf.gps(gps_feedback_1)

print(gps_vector_1)