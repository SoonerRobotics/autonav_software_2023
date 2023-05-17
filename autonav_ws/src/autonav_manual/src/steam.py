#!/usr/bin/env python3

import rclpy
import time
import threading
from steamcontroller import SteamController
from steamcontroller import SteamControllerInput
from enum import IntEnum
from autonav_msgs.msg import SteamInput
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
        self.log(f"Pressed {button} for {msTime}ms")
        self.log(f"Death -> {button == SteamControllerButton.B}")
        self.log(f"Manual -> {button == SteamControllerButton.START}")
        self.log(f"Autonomous -> {button == SteamControllerButton.STEAM}")
        self.log(f"Disabled -> {button == SteamControllerButton.BACK}")
        self.log(f"System State -> {self.getSystemState().state}")
        self.log(f"Device State -> {self.getDeviceState()}")
        self.log(f"System Mode -> {self.getSystemState().mode}")
        self.log("-------------------")
        if button == SteamControllerButton.B and msTime > 1500:
            self.log("go to death -_-")
            self.setSystemState(SystemStateEnum.SHUTDOWN)
            
        if button == SteamControllerButton.START and msTime > 1500 and self.getSystemState().state != SystemStateEnum.MANUAL:
            self.log("go to manual -_-")
            self.setSystemState(SystemStateEnum.MANUAL)
            
        if button == SteamControllerButton.STEAM and msTime > 1500 and self.getSystemState().state != SystemStateEnum.AUTONOMOUS:
            self.log("go to autonomous -_-")
            self.setSystemState(SystemStateEnum.AUTONOMOUS)
            
        if button == SteamControllerButton.BACK and msTime > 1500 and self.getSystemState().state != SystemStateEnum.DISABLED:
            self.log("go to disabled -_-")
            self.setSystemState(SystemStateEnum.DISABLED)

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
                self.buttons[button] = self.getClockNs()
            if self.buttons[button] > 0 and bool(sci.buttons & button) == False:
                if self.buttons[button] != 0:
                    self.onButtonReleased(button, (self.getClockNs() - self.buttons[button]) / 1000000)
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
 
