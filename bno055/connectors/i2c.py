from bno055.connectors.Connector import Connector
from rclpy.node import Node


class I2C(Connector):

    CONNECTIONTYPE_I2C = 'i2c'

    def __init__(self, node: Node):
        # Initialize parent
        super().__init__(node)

    def connect(self):
        # TODO implement IC2 integration
        raise NotImplementedError('I2C not yet implemented')

    def read(self, numberOfBytes):
        # TODO implement IC2 integration
        raise NotImplementedError('I2C not yet implemented')

    def write(self, data: bytearray):
        # TODO implement IC2 integration
        raise NotImplementedError('I2C not yet implemented')

