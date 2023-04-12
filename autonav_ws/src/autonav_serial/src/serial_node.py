#!/usr/bin/env python3

import rclpy
import can
import threading
import struct
from enum import Enum

from rclpy.node import Node
from autonav_msgs.msg import MotorInput, MotorFeedback
from autonav_libs import AutoNode, DeviceStateEnum as DeviceState, LogLevel, clamp


MOTOR_CONTROL_ID = 10
MOTOR_FEEDBACK_ID = 14


class SerialMotors(AutoNode):
    def __init__(self):
        super().__init__("autonav_serial_can")

    def setup(self):
        self.m_motorSubscriber = self.create_subscription(MotorInput, "/autonav/MotorInput", self.on_motor_input, 10)
        self.m_feedbackPublisher = self.create_publisher(MotorFeedback, "/autonav/MotorFeedback", 10)
        self.m_can = None
        self.m_canTimer = self.create_timer(0.5, self.canWorker)
        self.m_canReadThread = threading.Thread(target = self.canThreadWorker)
        self.m_canReadThread.daemon = True
        self.m_canReadThread.start()
    
    def canThreadWorker(self):
        while rclpy.ok():
            if self.getDeviceState() != DeviceState.READY and self.getDeviceState() != DeviceState.OPERATING:
                continue
            if self.m_can is not None:
                try:
                    msg = self.m_can.recv(timeout = 1)
                    if msg is not None:
                        self.onCanMessageReceived(msg)
                except can.CanError:
                    self.log(LogLevel.ERROR, "CAN Error")

    def onCanMessageReceived(self, msg):
        if msg.arbitration_id == MOTOR_FEEDBACK_ID:
            deltaX, deltaY, deltaTheta  = struct.unpack("hhh", msg.data)
            feedback = MotorFeedback()
            feedback.delta_theta = deltaTheta / 10000.0
            feedback.delta_y = deltaY / 10000.0
            feedback.delta_x = deltaX / 10000.0
            self.m_feedbackPublisher.publish(feedback)  

    def canWorker(self):
        try:
            with open("/dev/autonav-can-835", "r") as f:
                pass

            if self.m_can is not None:
                return

            self.m_can = can.ThreadSafeBus(bustype="slcan", channel="/dev/autonav-can-835", bitrate=100000)
            self.setDeviceState(DeviceState.READY)
            self.log("found cannable")
        except:
            if self.m_can is not None:
                self.m_can = None

            self.log("Failed to find cannable")
            if self.getDeviceState() != DeviceState.STANDBY:
                self.setDeviceState(DeviceState.STANDBY)

        if self.m_can is not None and self.getDeviceState() != DeviceState.READY:
            self.setDeviceState(DeviceState.READY)

    def on_motor_input(self, input: MotorInput):
        if self.getDeviceState() != DeviceState.OPERATING:
            return
        
        packed_data = struct.pack("hh", int(input.forward_velocity * 1000.0), int(input.angular_velocity * 1000.0))
        can_msg = can.Message(arbitration_id=MOTOR_CONTROL_ID, data=packed_data)

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
