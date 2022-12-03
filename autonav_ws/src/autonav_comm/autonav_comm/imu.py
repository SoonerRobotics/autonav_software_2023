import rclpy
from vnpy import *

from autonav_msgs.msg import ImuData
from rclpy.publisher import Publisher

imu_publisher: Publisher = None

def readImu():
    s = VnSensor()

    if(not s.connect()):
        s.connect('/dev/ttyUSB0', 115200)

    accelData = s.read_acceleration_measurements
    ypr = s.read_yaw_pitch_roll

    accel_x = accelData.x
    accel_y = accelData.y
    accel_z = accelData.z

    yaw = ypr.x
    pitch = ypr.y
    roll = ypr.z


def main(args=None):
    global imu_publisher
    rclpy.init(args=args)

    s = VnSensor()
    s.connect('/dev/ttyUSB0', 115200)

	# Mobility Start message
	# mobility_start = can.Message(arbitration_id=9)
	# write_can.send(mobility_start)

    node = rclpy.create_node("autonav_comm_imu")
    imu_publisher = node.create_subscription(ImuData, "/autonav/odemetry/imu", 20)


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
