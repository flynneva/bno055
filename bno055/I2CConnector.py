from bno055.Connector import Connector
from rclpy.node import Node


class I2CConnector(Connector):
    def __init__(self, node: Node):
        # Initialize parent
        super().__init__(self, node)
