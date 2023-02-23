#!/usr/bin/env python3

import rclpy
import time
from vnpy import *

from autonav_msgs.msg import IMUData
from autonav_msgs.msg import Log
from autonav_msgs.msg import GPSData
from rclpy.publisher import Publisher

imu_publisher: Publisher = None
log_publisher: Publisher = None
gps_publisher: Publisher = None

has_published_headers = False
sensor = VnSensor()

def on_read_imu():
    global has_published_headers, sensor

    if(not has_published_headers):
        log = Log()
        log.file = "imu_data"
        log.data = "time,accel_x,accel_y,accel_z,yaw,pitch,roll,angular_x,angular_y,angular_z,gps_fix,latitude,longitude,altitude"
        log_publisher.publish(log)
        has_published_headers = True
        return

    if(not sensor.is_connected):
        sensor.connect("/dev/autonav-imu-200", 115200)

    if(not sensor.is_connected):
        return

    acceleration = sensor.read_acceleration_measurements()
    angular_velocity = sensor.read_angular_rate_measurements()
    ypr = sensor.read_yaw_pitch_roll()
    sensor_register = sensor.read_gps_solution_lla()

    data = IMUData()
    data.accel_x = acceleration.x
    data.accel_y = acceleration.y
    data.accel_z = acceleration.z
    data.yaw = ypr.x
    data.pitch = ypr.y
    data.roll = ypr.z
    data.angular_x = angular_velocity.x
    data.angular_y = angular_velocity.y
    data.angular_z = angular_velocity.z
    imu_publisher.publish(data)

    gps = GPSData()
    gps.latitude = sensor_register.lla.x
    gps.longitude = sensor_register.lla.y
    gps.altitude = sensor_register.lla.z
    gps.gps_fix = sensor_register.gps_fix
    gps_publisher.publish(gps)

    # Get lat/long if the sensor has GPS fix
    log = Log()
    log.file = "imu_data"
    log.data = f"{round(time.time()*1000)},{data.accel_x},{data.accel_y},{data.accel_z},{data.yaw},{data.pitch},{data.roll},{angular_velocity.x},{angular_velocity.y},{angular_velocity.z},{sensor_register.gps_fix},{sensor_register.lla.x},{sensor_register.lla.y},{sensor_register.lla.y}"
    log_publisher.publish(log)

def main():
    global node, imu_publisher, log_publisher, gps_publisher

    rclpy.init()

    node = rclpy.create_node("autonav_serial_imu")
    imu_publisher = node.create_publisher(IMUData, "/autonav/imu", 20)
    gps_publisher = node.create_publisher(GPSData, "/autonav/gps", 20)
    log_publisher = node.create_publisher(Log, "/autonav/logging", 20)

    node.create_timer(1, on_read_imu)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()