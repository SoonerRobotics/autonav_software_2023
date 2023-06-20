#!/usr/bin/env python3

import rclpy
import time
import threading
from steamcontroller import SteamController
from steamcontroller import SteamControllerInput
from enum import IntEnum
from autonav_msgs.msg import SteamInput, SafetyLights
from scr_core.node import Node
from scr_core.state import DeviceStateEnum, SystemStateEnum


class SteamControllerButton(IntEnum):
    RPADTOUCH = 0b00010000000000000000000000000000
    LPADTOUCH = 0b00001000000000000000000000000000
    RPAD =      0b00000100000000000000000000000000
    LPAD =      0b00000010000000000000000000000000
    RGRIP =     0b00000001000000000000000000000000
    LGRIP =     0b00000000100000000000000000000000
    START =     0b00000000010000000000000000000000
    STEAM =     0b00000000001000000000000000000000
    BACK =      0b00000000000100000000000000000000
    A =         0b00000000000000001000000000000000
    X =         0b00000000000000000100000000000000
    B =         0b00000000000000000010000000000000
    Y =         0b00000000000000000001000000000000
    LB =        0b00000000000000000000100000000000
    RB =        0b00000000000000000000010000000000
    LT =        0b00000000000000000000001000000000
    RT =        0b00000000000000000000000100000000


def hexToRgb(color: str):
    if color[0] == "#":
        color = color[1:]
    return [int(color[0:2], 16), int(color[2:4], 16), int(color[4:6], 16)]


def toSafetyLights(autonomous: bool, eco: bool, mode: int, brightness: int, color: str) -> SafetyLights:
    pkg = SafetyLights()
    pkg.mode = mode
    pkg.autonomous = autonomous
    pkg.eco = eco
    pkg.brightness = brightness
    colorr = hexToRgb(color)
    pkg.red = colorr[0]
    pkg.green = colorr[1]
    pkg.blue = colorr[2]
    return pkg


class SteamTranslationNode(Node):
    def __init__(self):
        super().__init__("autonav_manual_steamtranslator")

    def configure(self):
        self.buttons = {}
        for button in SteamControllerButton:
            self.buttons[button] = 0
        
        self.steamThread = threading.Thread(target=self.startSteamController)
        self.steamThread.daemon = True
        self.steamThread.start()
        self.joyPublisher = self.create_publisher(SteamInput, "/autonav/joy/steam", 20)
        self.safetyLightsPublisher = self.create_publisher(SafetyLights, "/autonav/SafetyLights", 20)
        
    def transition(self, old, new):
        pass

    def startSteamController(self):
        try:
            self.sc = SteamController(callback=self.onSteamControllerInput)
            if self.sc._handle:
                self.setDeviceState(DeviceStateEnum.OPERATING)
            self.sc.run()
        except KeyboardInterrupt:
            self.setDeviceState(DeviceStateEnum.OFF)
            self.sc.close()
            pass
        finally:
            time.sleep(5)
            self.setDeviceState(DeviceStateEnum.STANDBY)
            self.startSteamController()
    
    def onButtonReleased(self, button: SteamControllerButton, msTime: float):
        if button == SteamControllerButton.B:
            self.setSystemState(SystemStateEnum.SHUTDOWN)
            
        if button == SteamControllerButton.START and self.getSystemState().state != SystemStateEnum.MANUAL:
            self.setSystemState(SystemStateEnum.MANUAL)
            self.safetyLightsPublisher.publish(toSafetyLights(False, False, 2, 100, "#FF6F00"))
            
        if button == SteamControllerButton.STEAM and self.getSystemState().state != SystemStateEnum.AUTONOMOUS:
            self.setSystemState(SystemStateEnum.AUTONOMOUS)
            
        if button == SteamControllerButton.BACK and self.getSystemState().state != SystemStateEnum.DISABLED:
            self.setSystemState(SystemStateEnum.DISABLED)
            self.safetyLightsPublisher.publish(toSafetyLights(False, False, 2, 100, "#A020F0"))

    def onSteamControllerInput(self, _, sci: SteamControllerInput):
        if self.getDeviceState() != DeviceStateEnum.OPERATING:
            return
        
        msg = SteamInput()
        msg.status = int(sci.status)
        msg.seq = int(sci.seq)
        msg.buttons = []
        for button in SteamControllerButton:
            msg.buttons.append(bool(sci.buttons & button))
            if self.buttons[button] == 0 and bool(sci.buttons & button):
                self.buttons[button] = self.getClockMs()
            if self.buttons[button] > 0 and bool(sci.buttons & button) == False:
                if self.buttons[button] != 0:
                    self.onButtonReleased(button, self.getClockMs() - self.buttons[button])
                self.buttons[button] = 0
        msg.ltrig = float(sci.ltrig) / 255
        msg.rtrig = float(sci.rtrig) / 255
        msg.lpad_x = float(sci.lpad_x) / 32768
        msg.lpad_y = float(sci.lpad_y) / 32768
        msg.rpad_x = float(sci.rpad_x) / 32768
        msg.rpad_y = float(sci.rpad_y) / 32768
        msg.gpitch = float(sci.gpitch)
        msg.groll = float(sci.groll)
        msg.gyaw = float(sci.gyaw)
        msg.q1 = float(sci.q1)
        msg.q1 = float(sci.q2)
        msg.q1 = float(sci.q3)
        msg.q1 = float(sci.q4)

        self.joyPublisher.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(SteamTranslationNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
 
