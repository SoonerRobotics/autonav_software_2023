#!/usr/bin/env python3

import rclpy
import can
import struct
from enum import Enum

from rclpy.node import Node
from autonav_msgs.msg import MotorInput
from autonav_libs import AutoNode, Device, DeviceStateEnum as DeviceState


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


MOTOR_CONTROL_ID = 10


class Registers(Enum):
    MOTOR_OFFSET = 0


class SerialMotors(AutoNode):
    def __init__(self):
        super().__init__(Device.SERIAL_IMU, "autonav_serial_motors")

        self.motor_subscriber = self.create_subscription(
            MotorInput, "/autonav/MotorInput", self.on_motor_input, 10)
        self.canbus = None
        self.config.writeFloat(Registers.MOTOR_OFFSET.value, 35.0)
        
        self.set_state(DeviceState.STANDBY)

    def on_motor_input(self, input: MotorInput):
        if self.canbus is None:
            self.canbus = can.ThreadSafeBus(bustype="slcan", channel="/dev/autonav-can-835", bitrate=100000)
        
            try:
                with open("/dev/autonav-can-835", "r") as f:
                    pass

                if self.state != DeviceState.OPERATING:
                    self.set_state(DeviceState.OPERATING)
            except:
                self.canbus = None
                if self.state != DeviceState.STANDBY:
                    self.set_state(DeviceState.STANDBY)

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
    global node, canbus

    rclpy.init()
    rclpy.spin(SerialMotors())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
