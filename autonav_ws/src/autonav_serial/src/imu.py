#!/usr/bin/env python3

import rclpy
import time
from vnpy import *

from autonav_msgs.msg import IMUData
from autonav_msgs.msg import Log
from rclpy.publisher import Publisher

imu_publisher: Publisher = None
log_publisher: Publisher = None

def on_read_imu():
    sensor = VnSensor()

    if(not sensor.is_connected):
        sensor.connect("/dev/autonav-imu-200", 115200)

    if(not sensor.is_connected):
        return

    acceleration = sensor.read_acceleration_measurements()
    ypr = sensor.read_yaw_pitch_roll()

    data = IMUData()
    data.accel_x = acceleration.x
    data.accel_y = acceleration.y
    data.accel_z = acceleration.z
    data.yaw = ypr.x
    data.pitch = ypr.y
    data.roll = ypr.z
    imu_publisher.publish(data)

    # Get lat/long if the sensor has GPS fix
    if(sensor.gps_fix):
        gps = sensor.read_gps_solution_lla()
        latitude = gps.latitude
        longitude = gps.longitude

    log = Log()
    log.file = "imu_data"
    bool_gps = 0 if sensor.gps_fix else 1
    latitude_value = 0 if sensor.gps_fix else latitude
    longitude_value = 0 if sensor.gps_fix else longitude
    log.data = f"{time.time()},{data.accel_x},{data.accel_y},{data.accel_z},{data.yaw},{data.pitch},{data.roll},{bool_gps},{latitude_value},{longitude_value}"
    log_publisher.publish(log)

def main():
    global node, imu_publisher, log_publisher

    rclpy.init()

    node = rclpy.create_node("autonav_serial_imu")
    imu_publisher = node.create_publisher(IMUData, "/autonav/odemetry/imu", 20)
    log_publisher = node.create_publisher(Log, "/autonav/logging", 20)

    log = Log()
    log.file = "imu_data"
    log.data = "time,accel_x,accel_y,accel_z,yaw,pitch,roll,gps_has_fix,latitude,longitude"
    log_publisher.publish(log)

    node.create_timer(0.5, on_read_imu)

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()