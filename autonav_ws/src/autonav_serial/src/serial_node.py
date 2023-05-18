#!/usr/bin/env python3

from ctypes import Structure, c_bool, c_uint8
import rclpy
import can
import threading
import struct
from autonav_msgs.msg import MotorInput, MotorFeedback, ObjectDetection, MotorControllerDebug, SafetyLights, Conbus
from scr_core.node import Node
from scr_core.state import DeviceStateEnum


MOTOR_CONTROL_ID = 10
ESTOP_ID = 0
MOBILITY_STOP_ID = 1
MOBILITY_START_ID = 9
MOTOR_FEEDBACK_ID = 14
OBJECT_DETECTION = 20
SAFETY_LIGHTS_ID = 13

CAN_50 = 50
CAN_51 = 51


class SafetyLightsPacket(Structure):
    _fields_ = [
        ("autonomous", c_bool, 1),
        ("eco", c_uint8, 1),
        ("mode", c_uint8, 6),
        ("brightness", c_uint8, 8),
        ("red", c_uint8, 8),
        ("green", c_uint8, 8),
        ("blue", c_uint8, 8)
    ]


class SerialMotors(Node):
    def __init__(self):
        super().__init__("autonav_serial_can")

        self.currentForwardVel = 0.0
        self.setpointForwardVel = 0.0
        self.currentAngularVel = 0.0
        self.setpointAngularVel = 0.0
        self.can = None
        self.lastMotorInput = None

    def configure(self):
        self.safetyLightsSubscriber = self.create_subscription(SafetyLights, "/autonav/SafetyLights", self.onSafetyLightsReceived, 20)
        self.motorInputSubscriber = self.create_subscription(MotorInput, "/autonav/MotorInput", self.onMotorInputReceived, 20)
        self.motorDebugPublisher = self.create_publisher(MotorControllerDebug, "/autonav/MotorControllerDebug", 20)
        self.objectDetectionPublisher = self.create_publisher(ObjectDetection, "/autonav/ObjectDetection", 20)
        self.motorFeedbackPublisher = self.create_publisher(MotorFeedback, "/autonav/MotorFeedback", 20)
        self.conbuSubscriber = self.create_subscription(Conbus, "/autonav/conbus", self.onConbusReceived, 20)
        self.conbusPublisher = self.create_publisher(Conbus, "/autonav/conbus", 20)
        self.canTimer = self.create_timer(0.5, self.canWorker)
        self.canReadThread = threading.Thread(target=self.canThreadWorker)
        self.canReadThread.daemon = True
        self.canReadThread.start()

    def transition(self, old, updated):
        return

    def canThreadWorker(self):
        while rclpy.ok():
            if self.getDeviceState() != DeviceStateEnum.READY and self.getDeviceState() != DeviceStateEnum.OPERATING:
                continue
            if self.can is not None:
                try:
                    msg = self.can.recv(timeout=1)
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
            self.motorFeedbackPublisher.publish(feedback)
            self.log(f"20,{self.getClockMs()},{feedback.delta_x},{feedback.delta_y},{feedback.delta_theta}")

        if arb_id == ESTOP_ID:
            self.log(f"Received ESTOP")
            self.setEStop(True)

        if arb_id == MOBILITY_STOP_ID:
            self.log(f"Received Mobiltiy Stop")
            self.setMobility(False)

        if arb_id == MOBILITY_START_ID:
            self.log(f"Received Mobility Start")
            self.setMobility(True)

        if arb_id == CAN_50:
            currentForwardVel, setpointForwardVel, currentAngularVel, setpointAngularVel = struct.unpack("hhhh", msg.data)
            self.currentForwardVel = currentForwardVel / 1000.0
            self.setpointForwardVel = setpointForwardVel / 1000.0
            self.currentAngularVel = currentAngularVel / 1000.0
            self.setpointAngularVel = setpointAngularVel / 1000.0

            # Log the CAN_50 message
            self.log(f"50,{self.getClockMs()},{self.currentForwardVel},{self.setpointForwardVel},{self.currentAngularVel},{self.setpointAngularVel}")

        if arb_id == CAN_51:
            leftMotorOutput, rightMotorOutput = struct.unpack("hh", msg.data)
            leftMotorOutput /= 1000.0
            rightMotorOutput /= 1000.0

            # Create a MotorControllerDebug message and publish it
            pkg = MotorControllerDebug()
            pkg.current_forward_velocity = self.currentForwardVel
            pkg.forward_velocity_setpoint = self.setpointForwardVel
            pkg.current_angular_velocity = self.currentAngularVel
            pkg.angular_velocity_setpoint = self.setpointAngularVel
            pkg.left_motor_output = leftMotorOutput
            pkg.right_motor_output = rightMotorOutput
            pkg.timestamp = self.getClockMs() * 1.0
            self.motorDebugPublisher.publish(pkg)

            # Log the CAN_51 message
            self.log(f"51,{self.getClockMs()},{leftMotorOutput},{rightMotorOutput}")

        if arb_id == OBJECT_DETECTION:
            # Load in 4 bytes
            zero, left, middle, right = struct.unpack("BBBB", msg.data)
            pkg = ObjectDetection()
            pkg.sensor_1 = left
            pkg.sensor_2 = middle
            pkg.sensor_3 = right
            self.objectDetectionPublisher.publish(pkg)
            
        if arb_id >= 1000 and arb_id <= 1300:
            pkg = Conbus()
            pkg.id = arb_id
            pkg.data = msg.data
            self.conbusPublisher.publish(pkg)
            self.log(f"Received CONBUS Instruction: {arb_id} -> {','.join([str(x) for x in msg.data])}")

    def canWorker(self):
        try:
            with open("/dev/autonav-can-835", "r") as f:
                pass

            if self.can is not None:
                return

            self.can = can.ThreadSafeBus(bustype="slcan", channel="/dev/autonav-can-835", bitrate=100000)
            self.setDeviceState(DeviceStateEnum.OPERATING)
        except:
            if self.can is not None:
                self.can = None

            if self.getDeviceState() != DeviceStateEnum.STANDBY:
                self.setDeviceState(DeviceStateEnum.STANDBY)

    def onSafetyLightsReceived(self, lights: SafetyLights):
        if self.getDeviceState() != DeviceStateEnum.OPERATING:
            return

        packed_data = SafetyLightsPacket()
        packed_data.autonomous = lights.autonomous
        packed_data.eco = lights.eco
        packed_data.mode = lights.mode
        packed_data.brightness = lights.brightness
        packed_data.red = lights.red
        packed_data.green = lights.green
        packed_data.blue = lights.blue
        can_msg = can.Message(arbitration_id=SAFETY_LIGHTS_ID, data=bytes(packed_data))
        self.log(f"Setting SafetyLights: {'Autonomous' if lights.autonomous else 'Manual'},{'Eco' if lights.eco else 'Not Eco'},{lights.mode},{lights.brightness},{lights.red},{lights.green},{lights.blue}")

        try:
            self.can.send(can_msg)
        except can.CanError:
            self.log("Failed to send SafetyLights CAN message")
            
    def onConbusReceived(self, instruction: Conbus):
        if self.getDeviceState() != DeviceStateEnum.OPERATING:
            return
        
        self.log(f"CONBUS Instruction: {instruction.id} -> {','.join([str(x) for x in instruction.data])}")
        can_msg = can.Message(arbitration_id = instruction.id, data = bytearray(instruction.data))
        try:
            self.can.send(can_msg)
        except can.CanError:
            self.log("Failed to send ConBus CAN message")  

    def onMotorInputReceived(self, input: MotorInput):
        if self.getDeviceState() != DeviceStateEnum.OPERATING:
            return

        packed_data = struct.pack("hh", int(input.forward_velocity * 1000.0), int(input.angular_velocity * 1000.0))
        can_msg = can.Message(arbitration_id=MOTOR_CONTROL_ID, data=packed_data)

        try:
            self.can.send(can_msg)
        except can.CanError:
            self.log("Failed to send MotorInput CAN message")


def main():
    rclpy.init()
    rclpy.spin(SerialMotors())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
