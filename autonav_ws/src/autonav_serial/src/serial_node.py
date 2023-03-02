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
        self.motor_subscriber = self.create_subscription(MotorInput, "/autonav/MotorInput", self.on_motor_input, 10)
        self.config.writeFloat(Registers.MOTOR_OFFSET.value, 35.0)
        self.canbus = None
        self.canbus_timer = self.create_timer(0.5, self.on_canbus_check)

    def on_canbus_check(self):
        if self.canbus is None:
            try:
                with open("/dev/autonav-can-835", "w") as f:
                    pass

                self.canbus = can.ThreadSafeBus(bustype="slcan", channel="/dev/autonav-can-835", bitrate=100000)
                self.log("CAN found, starting CAN bus", LogLevel.INFO)
                self.set_device_state(DeviceState.READY)
            except:
                if self.device_state != DeviceState.STANDBY:
                    self.log("No CAN found, retrying in 0.5 second(s)", LogLevel.WARNING)
                    self.set_device_state(DeviceState.STANDBY)
                return
            
        self.log(f"Switchy -> {self.device_state}", LogLevel.INFO)
        if self.canbus is not None and self.device_state.value != DeviceState.READY.value:
            self.log("Attempting to switch to ready state", LogLevel.INFO)
            self.set_device_state(DeviceState.READY)

    def on_motor_input(self, input: MotorInput):
        if self.device_state != DeviceState.OPERATING:
            return

        left_speed = int(input.left_motor *
                         self.config.readFloat(Registers.MOTOR_OFFSET.value))
        right_speed = int(input.right_motor *
                          self.config.readFloat(Registers.MOTOR_OFFSET.value))
        packed_data = struct.pack("hh", left_speed, right_speed)

        can_msg = can.Message(arbitration_id=MOTOR_CONTROL_ID, data=packed_data)

        try:
            self.canbus.send(can_msg)
        except can.CanError:
            print("Failed to send motor message :(")


def main():
    rclpy.init()
    rclpy.spin(SerialMotors())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
