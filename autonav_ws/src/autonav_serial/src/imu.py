#!/usr/bin/env python3

import rclpy
import time
from vnpy import *

from autonav_msgs.msg import IMUData
from autonav_msgs.msg import Log
from rclpy.publisher import Publisher

imu_publisher: Publisher = None
log_publisher: Publisher = None

has_published_headers = False

def on_read_imu():
    global has_published_headers
    sensor = VnSensor()

    if(not has_published_headers):
        log = Log()
        log.file = "imu_data"
        log.data = "time,accel_x,accel_y,accel_z,yaw,pitch,roll,gps_locked,latitude,longitude,altitude"
        log_publisher.publish(log)
        has_published_headers = True
        return

    if(not sensor.is_connected):
        sensor.connect("/dev/autonav-imu-200", 115200)

    if(not sensor.is_connected):
        return

    acceleration = sensor.read_acceleration_measurements()
    ypr = sensor.read_yaw_pitch_roll()
    sensor_register = sensor.read_gps_solution_lla()

    data = IMUData()
    data.accel_x = acceleration.x
    data.accel_y = acceleration.y
    data.accel_z = acceleration.z
    data.yaw = ypr.x
    data.pitch = ypr.y
    data.roll = ypr.z
    imu_publisher.publish(data)

    # Get lat/long if the sensor has GPS fix
    log = Log()
    log.file = "imu_data"
    log.data = f"{round(time.time()*1000)},{data.accel_x},{data.accel_y},{data.accel_z},{data.yaw},{data.pitch},{data.roll},{sensor_register.gps_fix},{sensor_register.lla.x},{sensor_register.lla.y},{sensor_register.lla.y}"
    log_publisher.publish(log)

def main():
    global node, imu_publisher, log_publisher

    rclpy.init()

    node = rclpy.create_node("autonav_serial_imu")
    imu_publisher = node.create_publisher(IMUData, "/autonav/odemetry/imu", 20)
    log_publisher = node.create_publisher(Log, "/autonav/logging", 20)

    node.create_timer(0.5, on_read_imu)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()