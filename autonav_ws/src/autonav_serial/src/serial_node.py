#!/usr/bin/env python3

import rclpy
import can
import time
import threading
import struct
from autonav_msgs.msg import MotorInput, MotorFeedback, ObjectDetection
from scr_core.node import Node
from scr_core.state import DeviceStateEnum


MOTOR_CONTROL_ID = 10
ESTOP_ID = 0
MOBILITY_STOP_ID = 1
MOBILITY_START_ID = 9
MOTOR_FEEDBACK_ID = 14
OBJECT_DETECTION = 20


class SerialMotors(Node):
    def __init__(self):
        super().__init__("autonav_serial_can")

    def configure(self):
        self.m_motorSubscriber = self.create_subscription(MotorInput, "/autonav/MotorInput", self.on_motor_input, 10)
        self.m_feedbackPublisher = self.create_publisher(MotorFeedback, "/autonav/MotorFeedback", 10)
        self.m_objectPublisher = self.create_publisher(ObjectDetection, "/autonav/ObjectDetection", 10)
        self.m_can = None
        self.lastMotorInput = None
        self.m_canTimer = self.create_timer(0.5, self.canWorker)
        self.m_canReadThread = threading.Thread(target=self.canThreadWorker)
        self.m_canReadThread.daemon = True
        self.m_canReadThread.start()

    def canThreadWorker(self):
        while rclpy.ok():
            if self.getDeviceState() != DeviceStateEnum.READY and self.getDeviceState() != DeviceStateEnum.OPERATING:
                continue
            if self.m_can is not None:
                try:
                    msg = self.m_can.recv(timeout=1)
                    if msg is not None:
                        self.onCanMessageReceived(msg)
                except can.CanError:
                    pass

    def onCanMessageReceived(self, msg):
        arb_id = msg.arbitration_id
        if arb_id == MOTOR_FEEDBACK_ID:
            deltaX, deltaY, deltaTheta = struct.unpack("hhh", msg.data)
            feedback = MotorFeedback()
            feedback.delta_theta = deltaTheta / 10000.0
            feedback.delta_y = deltaY / 10000.0
            feedback.delta_x = deltaX / 10000.0
            self.m_feedbackPublisher.publish(feedback)

        if arb_id == ESTOP_ID:
            self.setEStop(True)

        if arb_id == MOBILITY_STOP_ID:
            self.setMobility(False)

        if arb_id == MOBILITY_START_ID:
            self.setMobility(True)

    def canWorker(self):
        try:
            with open("/dev/autonav-can-835", "r") as f:
                pass

            if self.m_can is not None:
                return

            self.m_can = can.ThreadSafeBus(bustype="slcan", channel="/dev/autonav-can-835", bitrate=100000)
            self.setDeviceState(DeviceStateEnum.READY)
        except:
            if self.m_can is not None:
                self.m_can = None

            if self.getDeviceState() != DeviceStateEnum.STANDBY:
                self.setDeviceState(DeviceStateEnum.STANDBY)

        if self.m_can is not None and self.getDeviceState() != DeviceStateEnum.READY:
            self.setDeviceState(DeviceStateEnum.READY)

    def on_motor_input(self, input: MotorInput):
        if self.getDeviceState() != DeviceStateEnum.OPERATING:
            return

        packed_data = struct.pack("hh", int(input.forward_velocity * 1000.0), int(input.angular_velocity * 1000.0))
        can_msg = can.Message(arbitration_id=MOTOR_CONTROL_ID, data=packed_data)

        try:
            self.m_can.send(can_msg)
            
            if self.lastMotorInput is None or self.lastMotorInput.forward_velocity != input.forward_velocity or self.lastMotorInput.angular_velocity != input.angular_velocity:
                self.lastMotorInput = input
                self.log(f"{time.time()},{input.forward_velocity},{input.angular_velocity}")
        except can.CanError:
            print("Failed to send motor message :(")


def main():
    rclpy.init()
    rclpy.spin(SerialMotors())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
