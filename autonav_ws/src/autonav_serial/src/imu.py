#!/usr/bin/env python3

import rclpy
import time
import threading
from vnpy import *
from autonav_msgs.msg import IMUData
from autonav_msgs.msg import GPSFeedback
from scr_msgs.msg import SystemState
from scr_core.node import Node
from scr_core.state import DeviceStateEnum, SystemStateEnum

IMU_READ_RATE = 0
IMU_NOTFOUND_RETRY = 1
IMU_BADCONNECT_RETRY = 2


class IMUNode(Node):
    def __init__(self):
        super().__init__("autonav_serial_imu")

    def configure(self):
        self.m_sensor = VnSensor()
        self.m_hasPublishedHeaders = False

        self.config.setFloat(IMU_READ_RATE, 0.1)
        self.config.setFloat(IMU_NOTFOUND_RETRY, 5.0)
        self.config.setFloat(IMU_BADCONNECT_RETRY, 5.0)

        self.m_imuPublisher = self.create_publisher(IMUData, "/autonav/imu", 20)
        self.m_gpsPublisher = self.create_publisher(GPSFeedback, "/autonav/gps", 20)

        self.m_imuThread = threading.Thread(target=self.imuWorker)
        self.m_imuThread.start()

    def transition(self, old: SystemState, updated: SystemState):
        return

    def imuWorker(self):
        while rclpy.ok() and self.getSystemState() != SystemStateEnum.SHUTDOWN:
            if (not self.m_hasPublishedHeaders):
                self.m_hasPublishedHeaders = True
                continue

            if (not self.m_sensor.is_connected):
                try:
                    with open("/dev/autonav-imu-200", "r") as f:
                        pass

                    self.m_sensor.connect("/dev/autonav-imu-200", 115200)
                    self.setDeviceState(DeviceStateEnum.OPERATING)
                except:
                    self.setDeviceState(DeviceStateEnum.STANDBY)
                    time.sleep(self.config.getFloat(IMU_NOTFOUND_RETRY))
                    continue

            if (not self.m_sensor.is_connected):
                time.sleep(self.config.getFloat(IMU_BADCONNECT_RETRY))
                self.setDeviceState(DeviceStateEnum.STANDBY)
                continue

            if self.getDeviceState() != DeviceStateEnum.OPERATING:
                continue

            acceleration = self.m_sensor.read_acceleration_measurements()
            angular_velocity = self.m_sensor.read_angular_rate_measurements()
            ypr = self.m_sensor.read_yaw_pitch_roll()
            sensor_register = self.m_sensor.read_gps_solution_lla()

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
            self.m_imuPublisher.publish(data)

            gps = GPSFeedback()
            gps.latitude = sensor_register.lla.x
            gps.longitude = sensor_register.lla.y
            gps.altitude = sensor_register.lla.z
            gps.gps_fix = sensor_register.gps_fix
            gps.satellites = sensor_register.num_sats
            self.m_gpsPublisher.publish(gps)
            self.log(f"{time.time()},{data.accel_x},{data.accel_y},{data.accel_z},{data.yaw},{data.pitch},{data.roll},{data.angular_x},{data.angular_y},{data.angular_z},{gps.latitude},{gps.longitude},{gps.altitude},{gps.gps_fix},{gps.satellites}")

            # Get lat/long if the sensor has GPS fix
            time.sleep(self.config.getFloat(IMU_READ_RATE))


def main():
    rclpy.init()
    rclpy.spin(IMUNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
