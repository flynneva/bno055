from bno055.connectors.Connector import Connector
from rclpy.node import Node


class I2CConnector(Connector):

    CONNECTIONTYPE_I2C = 'i2c'

    def __init__(self, node: Node):
        # Initialize parent
        super().__init__(node)
