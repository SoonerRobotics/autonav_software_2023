#!/usr/bin/env python3

import rclpy
import time
import os
import threading
from vnpy import *
from autonav_msgs.msg import IMUData
from autonav_msgs.msg import GPSFeedback
from scr_core.node import Node
from scr_core.state import DeviceStateEnum, SystemStateEnum

IMU_READ_RATE = "imu_read_rate"


class IMUNode(Node):
    def __init__(self):
        super().__init__("autonav_serial_imu")

    def configure(self):
        self.vectorNavSensor = VnSensor()

        self.config.setFloat(IMU_READ_RATE, 0.1)

        self.imuPublisher = self.create_publisher(IMUData, "/autonav/imu", 20)
        self.gpsPublisher = self.create_publisher(GPSFeedback, "/autonav/gps", 20)

        self.imuThread = threading.Thread(target=self.imuWorker)
        self.imuThread.daemon = True
        self.imuThread.start()

    def transition(self, old, updated):
        return

    def imuWorker(self):
        while rclpy.ok() and self.getSystemState().state != SystemStateEnum.SHUTDOWN:
            if (not self.vectorNavSensor.is_connected):
                try:
                    if not os.path.exists("/dev/autonav-imu-200"):
                        time.sleep(3.0)
                        continue

                    self.vectorNavSensor.connect("/dev/autonav-imu-200", 115200)
                    self.setDeviceState(DeviceStateEnum.OPERATING)
                except:
                    self.setDeviceState(DeviceStateEnum.STANDBY)
                    time.sleep(3.0)
                    continue

            if (not self.vectorNavSensor.is_connected):
                time.sleep(3.0)
                self.setDeviceState(DeviceStateEnum.STANDBY)
                continue

            if self.getDeviceState() != DeviceStateEnum.OPERATING:
                continue

            acceleration = self.vectorNavSensor.read_acceleration_measurements()
            angular_velocity = self.vectorNavSensor.read_angular_rate_measurements()
            ypr = self.vectorNavSensor.read_yaw_pitch_roll()
            sensor_register = self.vectorNavSensor.read_gps_solution_lla()

            imu = IMUData()
            imu.accel_x = acceleration.x
            imu.accel_y = acceleration.y
            imu.accel_z = acceleration.z
            imu.yaw = ypr.x
            imu.pitch = ypr.y
            imu.roll = ypr.z
            imu.angular_x = angular_velocity.x
            imu.angular_y = angular_velocity.y
            imu.angular_z = angular_velocity.z
            self.imuPublisher.publish(imu)

            gps = GPSFeedback()
            gps.latitude = sensor_register.lla.x
            gps.longitude = sensor_register.lla.y
            gps.altitude = sensor_register.lla.z
            gps.gps_fix = sensor_register.gps_fix
            gps.satellites = sensor_register.num_sats
            self.gpsPublisher.publish(gps)
            time.sleep(self.config.getFloat(IMU_READ_RATE))


def main():
    rclpy.init()
    rclpy.spin(IMUNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
