#!/usr/bin/env python3

import rclpy
import can
import struct
from enum import Enum

from rclpy.node import Node
from autonav_msgs.msg import MotorInput
from autonav_libs import AutoNode, Device, DeviceStateEnum as DeviceState, LogLevel


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


MOTOR_CONTROL_ID = 10


class Registers(Enum):
    MOTOR_OFFSET = 0


class SerialMotors(AutoNode):
    def __init__(self):
        super().__init__(Device.SERIAL_CAN, "autonav_serial_can")

    def setup(self):
        self.m_motorSubscriber = self.create_subscription(
            MotorInput, "/autonav/MotorInput", self.on_motor_input, 10)
        self.config.writeFloat(Registers.MOTOR_OFFSET.value, 35.0)
        self.m_can = None
        self.m_canTimer = self.create_timer(0.5, self.canWorker)

    def canWorker(self):
        try:
            with open("/dev/autonav-can-835", "r") as f:
                pass

            if self.m_can is not None:
                return

            self.m_can = can.ThreadSafeBus(
                bustype="slcan", channel="/dev/autonav-can-835", bitrate=100000)
            self.setDeviceState(DeviceState.READY)
        except:
            if self.m_can is not None:
                self.m_can = None

            if self.getDeviceState() != DeviceState.STANDBY:
                self.setDeviceState(DeviceState.STANDBY)

        if self.m_can is not None and self.getDeviceState() != DeviceState.READY:
            self.setDeviceState(DeviceState.READY)

    def on_motor_input(self, input: MotorInput):
        if self.getDeviceState() != DeviceState.OPERATING:
            return

        left_speed = int(input.left_motor *
                         self.config.readFloat(Registers.MOTOR_OFFSET.value))
        right_speed = int(input.right_motor *
                          self.config.readFloat(Registers.MOTOR_OFFSET.value))
        packed_data = struct.pack("hh", left_speed, right_speed)

        can_msg = can.Message(
            arbitration_id=MOTOR_CONTROL_ID, data=packed_data)

        try:
            self.m_can.send(can_msg)
        except can.CanError:
            print("Failed to send motor message :(")


def main():
    rclpy.init()
    rclpy.spin(SerialMotors())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
