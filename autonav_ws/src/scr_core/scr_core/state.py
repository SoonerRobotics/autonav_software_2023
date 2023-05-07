from enum import IntEnum


class DeviceStateEnum(IntEnum):
    OFF = 0
    STANDBY = 1
    READY = 2
    OPERATING = 3


class SystemStateEnum(IntEnum):
    DISABLED = 0
    AUTONOMOUS = 1
    MANUAL = 2
    SHUTDOWN = 3


class SystemMode(IntEnum):
    COMPETITION = 0
    SIMULATION = 1
    PRACTICE = 2