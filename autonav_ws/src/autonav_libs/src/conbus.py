from enum import Enum


class Opcode(Enum):
    READ = 0
    WRITE = 1
    READ_ALL = 2


class Device(Enum):
    MOTOR_CONTROLLER = 0,
    ESTOP_RECEIVER = 1,
    SAFETY_LIGHTS = 2


class Controller():
    def __init__(self):
        self.registers = [0] * 100

    def read(self, register):
        if register < 0 or register >= len(self.registers):
            raise ValueError("Register out of range")

        return self.registers[register]

    def write(self, register, value):
        if register < 0 or register >= len(self.registers):
            raise ValueError("Register out of range")

        self.registers[register] = value

    def read_all(self):
        return self.registers

    def __getitem__(self, key):
        return self.registers[key]
    
    def __setitem__(self, key, value):
        self.registers[key] = value