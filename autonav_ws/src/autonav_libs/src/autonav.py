from enum import Enum

class ConBusAddress(Enum):
    MANUAL_CONTROL = 1
    IMU_PUBLISH_SPEED = 2

class ConBusOpcode(Enum):
    READ = 0
    WRITE = 1
    READ_ALL = 2