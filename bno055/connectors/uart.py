# Connector for UART integration of the BNO-055
# See also https://pyserial.readthedocs.io/en/latest/pyserial_api.html
import sys

from bno055.connectors.Connector import Connector
from rclpy.node import Node
import serial


class UART(Connector):
    """Connector implementation for serial UART connection to the sensor."""

    CONNECTIONTYPE_UART = 'uart'

    def __init__(self, node: Node, baudrate, port, timeout):
        # Initialize parent class
        super().__init__(node)

        self.node = node
        self.baudrate = baudrate
        self.port = port
        self.timeout = timeout
        self.serialConnection = None

    def connect(self):
        self.node.get_logger().info('Opening serial port: "%s"...' % self.port)

        try:
            self.serialConnection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.serialutil.SerialException:
            self.node.get_logger().info('Unable to connect to IMU at port ' + self.port
                                        + '. Check to make sure your device is connected.')
            sys.exit(1)

    def read(self, numberOfBytes):
        return self.serialConnection.read(numberOfBytes)

    def write(self, data: bytearray):
        self.serialConnection.write(data)
