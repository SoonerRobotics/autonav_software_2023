#!/usr/bin/env python3


from enum import Enum

from autonav_msgs.msg import ConBusInstruction


class Opcode(Enum):
    READ = 0
    READ_ACK = 1
    WRITE = 2
    WRITE_ACK = 3
    READ_ALL = 4

class Controller():
    def __init__(self, id, publisher):
        self.registers = [0] * 100
        self.id = id
        self.publisher = publisher

    def on_instruction(self, instruction: ConBusInstruction):
        if instruction.device != self.id:
            return

        if instruction.opcode == Opcode.READ:
            self.read(instruction)
        elif instruction.opcode == Opcode.WRITE:
            self.write(instruction)
        elif instruction.opcode == Opcode.READ_ALL:
            self.read_all()

    def read(self, message):
        self.publisher.publish(ConBusInstruction(
            device=self.id,
            opcode=Opcode.READ_ACK,
            register=message.register,
            value=self.registers[message.register]
        ))

    def write(self, instruction: ConBusInstruction):
        self.registers[instruction.register] = instruction.value
        self.publisher.publish(ConBusInstruction(
            device=self.id,
            opcode=Opcode.WRITE_ACK,
            register=instruction.register,
            value=instruction.value
        ))

    def read_all(self):
        for i in range(len(self.registers)):
            self.publisher.publish(ConBusInstruction(
                device=self.id,
                opcode=Opcode.READ_ACK,
                register=i,
                value=self.registers[i]
            ))

    def intToUint8s(self, value):
        return [
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF
        ]

    def floatToUint8s(self, value):
        large = value * 100000
        return self.intToUint8s(large)

    def uint8sToInt(self, bytes):
        return (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3]

    def uint8sToFloat(self, bytes):
        large = self.uint8sToInt(bytes)
        return large / 100000

    def readInt(self, register):
        return self.uint8sToInt(self.registers[register])

    def readFloat(self, register):
        return self.uint8sToFloat(self.registers[register])

    def writeInt(self, register, value):
        self.registers[register] = self.intToUint8s(value)

    def writeFloat(self, register, value):
        self.registers[register] = self.floatToUint8s(value)

    def __getitem__(self, key):
        return self.registers[key]

    def __setitem__(self, key, value):
        self.registers[key] = value
