from scr_msgs.msg import ConfigurationInstruction
from rclpy.node import Node
import struct


GET = 0
SET = 1
GET_ACK = 2
SET_ACK = 3
GET_ALL = 4


class Configuration:
    def __init__(self, id, node: Node):
        self.id = id
        self.node = node
        self.subscriber = node.create_subscription(ConfigurationInstruction, "/scr/configuration", self.onConfigurationInstruction, 20)
        self.publisher = node.create_publisher(ConfigurationInstruction, "/scr/configuration", 20)

        # a map of key to map of key to byte array
        self.cache = {}
        self.cache[self.id] = {}

    def getInt(self, address):
        return self.getIntFrom(self.id, address)

    def getIntFrom(self, device, address):
        return int.from_bytes(self.cache[device][address], byteorder='big', signed=True)

    def setInt(self, address, value: int):
        self.setIntTo(self.id, address, value)

    def setIntTo(self, device, address, value: int):
        if device not in self.cache:
            self.cache[device] = {}
        self.cache[device][address] = value.to_bytes(
            4, byteorder='big', signed=True)
        instruction = ConfigurationInstruction()
        instruction.device = device
        instruction.opcode = SET_ACK
        instruction.address = address
        instruction.data = self.cache[device][address]
        self.publisher.publish(instruction)

    def getFloat(self, address):
        return self.getFloatFrom(self.id, address)

    def getFloatFrom(self, device, address):
        return struct.unpack('f', self.cache[device][address])[0]

    def setFloat(self, address, value: float):
        self.setFloatTo(self.id, address, value)

    def setFloatTo(self, device, address, value: float):
        if device not in self.cache:
            self.cache[device] = {}
        self.cache[device][address] = bytearray(struct.pack('f', value))
        instruction = ConfigurationInstruction()
        instruction.device = device
        instruction.opcode = SET_ACK
        instruction.address = address
        instruction.data = self.cache[device][address]
        self.publisher.publish(instruction)

    def getBool(self, address):
        return self.getBoolFrom(self.id, address)

    def getBoolFrom(self, device, address):
        return bool.from_bytes(self.cache[device][address], byteorder='big')

    def setBool(self, address, value: bool):
        self.setBoolTo(self.id, address, value)

    def setBoolTo(self, device, address, value: bool):
        if device not in self.cache:
            self.cache[device] = {}
        self.cache[device][address] = bytes([value])
        instruction = ConfigurationInstruction()
        instruction.device = device
        instruction.opcode = SET_ACK
        instruction.address = address
        instruction.data = self.cache[device][address]
        self.publisher.publish(instruction)

    def recache(self):
        self.cache = {}

        instruction = ConfigurationInstruction()
        instruction.device = self.id
        instruction.opcode = GET_ALL
        self.publisher.publish(instruction)

    def onConfigurationInstruction(self, instruction: ConfigurationInstruction):
        amTarget = instruction.device == self.id

        if instruction.opcode == GET and amTarget:
            response = ConfigurationInstruction()
            response.device = self.id
            response.opcode = GET_ACK
            response.address = instruction.address
            response.data = self.cache[instruction.device][instruction.address]
            self.publisher.publish(response)

        if instruction.opcode == SET and amTarget:
            self.cache[instruction.device][instruction.address] = instruction.data
            response = ConfigurationInstruction()
            response.device = self.id
            response.opcode = SET_ACK
            response.address = instruction.address
            response.data = self.cache[instruction.device][instruction.address]
            self.publisher.publish(response)

        if instruction.opcode == SET_ACK or instruction.opcode == GET_ACK:
            if instruction.device not in self.cache:
                self.cache[instruction.device] = {}
            self.cache[instruction.device][instruction.address] = instruction.data

        if instruction.opcode == GET_ALL and not amTarget and self.id in self.cache:
            for address in self.cache[self.id]:
                response = ConfigurationInstruction()
                response.device = self.id
                response.opcode = GET_ACK
                response.address = address
                response.data = self.cache[self.id][address]
                self.publisher.publish(response)
