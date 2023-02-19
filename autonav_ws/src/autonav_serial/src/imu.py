#!/usr/bin/env python3

import rclpy
from vnpy import *

from autonav_msgs.msg import IMUData
from autonav_msgs.msg import Log
from rclpy.publisher import Publisher

imu_publisher: Publisher = None
log_publisher: Publisher = None

def on_read_imu():
    sensor = VmSensor()

    if(not sensor.is_connected):
        sensor.connect("/dev/ttyUSB0", 115200)

    acceleration = s.read_acceleration_measurements()
    ypr = s.read_yaw_pitch_roll()

    data = IMUData()
    data.accel_x = acceleration.accel_x
    data.accel_y = acceleration.accel_y
    data.accel_z = acceleration.accel_z
    data.yaw = ypr.x
    data.pitch = ypr.y
    data.roll = ypr.z
    imu_publisher.publish(data)

    log = Log()
    log.file = "imu_data"
    log.data = f"{dta.accel_x},{data.accel_y},{data.accel_z},{data.yaw},{data.pitch},{data.roll}"
    log_publisher.publish(log)

def main():
    global node, imu_publisher

    rclpy.init()

    node = rclpy.create_node("autonav_serial")
    imu_publisher = node.create_publisher(MotorInput, "/autonav/odemetry/imu", 20)
    log_publisher = node.create_publisher(Log, "/autonav/logging", 20)

    node.create_timer(0.5, on_read_imu())

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()