from autonav_msgs.msg import ConfigurationInstruction
from rclpy.node import Node
from enum import IntEnum


class Opcode(IntEnum):
    GET = 0
    SET = 1
    GET_ACK = 2
    SET_ACK = 3
    GET_ALL = 4


class Configuration:
    def __init__(self, id, node: Node):
        self.id = id
        self.subscriber = node.create_subscription(ConfigurationInstruction, "/autonav/configuration", self.onConfigurationInstruction, 10)
        self.publisher = node.create_publisher(ConfigurationInstruction, "/autonav/configuration", 10)

        # a map of key to map of key to byte array
        self.cache = {}

    def fromBytes(self, value: bytes, type: type):
        if type == int:
            return int.from_bytes(value, byteorder='big')
        if type == float:
            return float.from_bytes(value, byteorder='big')
        if type == str:
            return value.decode('utf-8')
        if type == bytes:
            return value
        if type == bool:
            return value[0] != 0
        return None

    def toBytes(self, value: type):
        if type == int:
            return value.to_bytes(4, byteorder='big')
        if type == float:
            return value.to_bytes(4, byteorder='big')
        if type == str:
            return value.encode('utf-8')
        if type == bytes:
            return value
        if type == bool:
            return bytes([1 if value else 0])
        return None

    def get(self, address: int, type: type):
        return self.get(self.id, address, type, None)

    def get(self, address: int, type: type, default: type):
        return self.get(self.id, address, type, default)

    def get(self, device, address: int, type: type):
        return self.get(device, address, type, None)

    def get(self, device, address: int, type: type, default: type):
        if device in self.cache and address in self.cache[device]:
            return self.fromBytes(self.cache[device][address], type)
        return default

    def set(self, address: int, value: type):
        self.set(self.id, address, value)

    def set(self, device, address: int, value: type):
        self.cache[device][address] = self.toBytes(value)

    def recache(self):
        self.cache = {}
        
        instruction = ConfigurationInstruction()
        instruction.device = self.id
        instruction.opcode = Opcode.GET_ALL
        self.publisher.publish(instruction)

    def onConfigurationInstruction(self, instruction: ConfigurationInstruction):
        amTarget = instruction.device == self.id

        if instruction.opcode == Opcode.GET and amTarget:
            response = ConfigurationInstruction()
            response.device = self.id
            response.opcode = Opcode.GET_ACK
            response.address = instruction.address
            response.data = self.cache[instruction.device][instruction.address]
            self.publisher.publish(response)

        if instruction.opcode == Opcode.SET and amTarget:
            self.cache[instruction.device][instruction.address] = instruction.data
            response = ConfigurationInstruction()
            response.device = self.id
            response.opcode = Opcode.SET_ACK
            response.address = instruction.address
            response.data = self.cache[instruction.device][instruction.address]
            self.publisher.publish(response)

        if instruction.opcode == Opcode.SET_ACK or instruction.opcode == Opcode.GET_ACK:
            self.cache[instruction.device][instruction.address] = instruction.data

        if instruction.opcode == Opcode.GET_ALL and not amTarget:
            for device in self.cache:
                for address in self.cache[device]:
                    response = ConfigurationInstruction()
                    response.device = self.id
                    response.opcode = Opcode.GET_ACK
                    response.address = address
                    response.data = self.cache[device][address]
                    self.publisher.publish(response)
