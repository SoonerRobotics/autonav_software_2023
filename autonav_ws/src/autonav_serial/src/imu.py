#!/usr/bin/env python3

import rclpy
import time
import threading
from vnpy import *
from enum import IntEnum

from autonav_msgs.msg import IMUData
from autonav_msgs.msg import GPSFeedback

from autonav_libs import AutoNode, DeviceStateEnum as DeviceState, SystemStateEnum as SystemState

class Registers(IntEnum):
    IMU_READ_RATE = 0
    IMU_NOTFOUND_RETRY = 1
    IMU_BADCONNECT_RETRY = 2


class IMUNode(AutoNode):
    def __init__(self):
        super().__init__("autonav_serial_imu")
        
    def setup(self):
        self.m_sensor = VnSensor()
        self.m_hasPublishedHeaders = False

        self.config.writeFloat(Registers.IMU_READ_RATE, 0.1)
        self.config.writeFloat(Registers.IMU_NOTFOUND_RETRY, 5.0)
        self.config.writeFloat(Registers.IMU_BADCONNECT_RETRY, 5.0)

        self.m_imuPublisher = self.create_publisher(IMUData, "/autonav/imu", 20)
        self.m_gpsPublisher = self.create_publisher(GPSFeedback, "/autonav/gps", 20)

        self.m_imuThread = threading.Thread(target=self.imuWorker)
        self.m_imuThread.start()

    def shutdown(self):
        self.m_imuThread.join()

    def imuWorker(self):
        while rclpy.ok() and self.getSystemState() != SystemState.SHUTDOWN:
            if (not self.m_hasPublishedHeaders):
                self.m_hasPublishedHeaders = True
                continue

            if (not self.m_sensor.is_connected):
                try:
                    with open("/dev/autonav-imu-200", "r") as f:
                        pass
                    
                    self.m_sensor.connect("/dev/autonav-imu-200", 115200)
                except:
                    self.setDeviceState(DeviceState.STANDBY)
                    time.sleep(self.config.readFloat(Registers.IMU_NOTFOUND_RETRY.value))
                    continue

            if (not self.m_sensor.is_connected):
                time.sleep(self.config.readFloat(Registers.IMU_BADCONNECT_RETRY.value))
                self.setDeviceState(DeviceState.STANDBY)
                continue

            if (self.getDeviceState() != DeviceState.READY):
                time.sleep(0.5)
                self.setDeviceState(DeviceState.READY)
                continue

            if self.getDeviceState() != DeviceState.OPERATING:
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
            self.m_gpsPublisher.publish(gps)

            # Get lat/long if the sensor has GPS fix
            time.sleep(self.config.readFloat(Registers.IMU_READ_RATE.value))


def main():
    rclpy.init()
    rclpy.spin(IMUNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
