from enum import Enum

from autonav_msgs.msg import ConBusInstruction


class Opcode(Enum):
    READ = 0
    READ_ACK = 1
    WRITE = 2
    WRITE_ACK = 3
    READ_ALL = 4


class Device(Enum):
    MOTOR_CONTROLLER = 0,
    ESTOP_RECEIVER = 1,
    SAFETY_LIGHTS = 2


class Controller():
    def __init__(self, id, publisher):
        self.registers = [0] * 100
        self.id = id
        self.publisher = publisher

    def read(self, message):
        if message.device != self.id:
            return

        self.publisher.publish(ConBusInstruction(
            device=self.id,
            opcode=Opcode.READ_ACK,
            register=message.register,
            value=self.registers[message.register]
        ))

    def write(self, instruction: ConBusInstruction):
        if instruction.device != self.id:
            return

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

    def __getitem__(self, key):
        return self.registers[key]

    def __setitem__(self, key, value):
        self.registers[key] = value
