import rclpy
from vnpy import *

from autonav_msgs.msg import ImuData
from rclpy.publisher import Publisher

imu_publisher: Publisher = None

def readImu():
    s = VnSensor()

    isConnected = s.is_connected
    if(not isConnected):
        s.connect('/dev/ttyUSB0', 115200)

    accelData = s.read_acceleration_measurements()
    ypr = s.read_yaw_pitch_roll()

    data = ImuData()

    data.accel_x = accelData.x
    data.accel_y = accelData.y
    data.accel_z = accelData.z

    data.yaw = ypr.x
    data.pitch = ypr.y
    data.roll = ypr.z

    imu_publisher.publish(data)


def main(args=None):
    global imu_publisher
    rclpy.init(args=args)

    node = rclpy.create_node("autonav_comm_imu")
    imu_publisher = node.create_publisher(ImuData, "/autonav/odemetry/imu", 20)

    imuUpdateTimer = node.create_timer(0.5, readImu)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
	main()