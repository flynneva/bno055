# Connector for UART integration of the BNO-055
# See also https://pyserial.readthedocs.io/en/latest/pyserial_api.html

import serial
import sys
from rclpy.node import Node

from bno055.Connector import Connector

class UARTConnector(Connector):

    CONNECTIONTYPE_UART = 'uart'

    def __init__(self, node: Node, baudrate, port, timeout):
        # Initialize parent
        super().__init__(node)
        self.node = node
        self.baudrate = baudrate
        self.port = port
        self.timeout = timeout

    def connect(self):
        self.node.get_logger().info('Opening serial port: "%s"...' % self.port)
        #usb_con = open_serial(port, baudrate, 0.02)

        try:
            self.conn = serial.Serial(self.port, self.baudrate, self.timeout)
        except serial.serialutil.SerialException:
            self.node.get_logger().info("Unable to connect to IMU at port " + self.port + ". Check to make sure your device is connected.")
            sys.exit(1)

    def receive(self, register, numBytes=1):
        # TODO
        pass


