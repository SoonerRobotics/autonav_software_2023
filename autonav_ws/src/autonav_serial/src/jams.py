#!/usr/bin/env python3

from scr_core.state import DeviceStateEnum, SystemStateEnum
from scr_msgs.msg import SystemState
from scr_core.node import Node
from pydub import AudioSegment
from pydub.playback import _play_with_simpleaudio
import rclpy
import os


CONFIG_MUSIC_OPTION = "option"
CONFIG_MUSIC_VOLUME = "volume"
CONFIG_MUSIC_DISABLED = "disabled"
AUDIO_FILES = ["oufightsong.wav", "peaches.wav", "goodmorning.wav"]


class Jammies(Node):
    def __init__(self):
        super().__init__("autonav_serial_jams")

    def configure(self):
        self.config.setInt(CONFIG_MUSIC_OPTION, 0)
        self.config.setFloat(CONFIG_MUSIC_VOLUME, -30.0)
        self.config.setBool(CONFIG_MUSIC_DISABLED, False)
        self.setDeviceState(DeviceStateEnum.READY)
        self.segment = None
        self.audio = None
        
    def getPathToMusic(self, name):
        homedir = os.path.expanduser("~")
        return os.path.abspath(f"{homedir}/autonav_software_2023/deps/songs/{name}")

    def playMusic(self):
        if self.config.getBool(CONFIG_MUSIC_DISABLED):
            return
        
        self.segment = AudioSegment.from_wav(self.getPathToMusic(AUDIO_FILES[self.config.getInt(CONFIG_MUSIC_OPTION)]))
        self.segment = self.segment + self.config.getFloat(CONFIG_MUSIC_VOLUME)
        if self.config.getInt(CONFIG_MUSIC_OPTION) == 0:
            self.segment = self.segment[2000:]
        elif self.config.getInt(CONFIG_MUSIC_OPTION) == 1:
            self.segment = self.segment[24000:]
        elif self.config.getInt(CONFIG_MUSIC_OPTION) == 2:
            self.segment = self.segment[750:]
        self.segment = self.segment * 50 # Make sure this bad boy never stops playing unless we want it to
        self.audio = _play_with_simpleaudio(self.segment)

    def stopMusic(self):
        if self.segment is None:
            return
        
        try:
            self.audio.stop()
        except:
            return
            

    def transition(self, old: SystemState, updated: SystemState):
        if updated.state == SystemStateEnum.AUTONOMOUS and old.state != SystemStateEnum.AUTONOMOUS:
            self.playMusic()
        else:
            self.stopMusic()


def main():
    rclpy.init()
    rclpy.spin(Jammies())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
