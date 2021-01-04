from connectors import Connector
import serial
import sys


class UARTConnector(Connector):

    def __init__(self, node: Node, baudrate, port, timeout):
        self.node = node
        self.baudrate = baudrate
        self.port = port
        self.timeout = timeout

    def connect(self):
        self.node.get_logger().info('Opening serial port: "%s"...' % self.port)
        #usb_con = open_serial(port, baudrate, 0.02)

        try:
            self.conn = serial.Serial(self.port, self.baudrate, self.timeout_)
        except serial.serialutil.SerialException:
            self.node.get_logger().info("Unable to connect to IMU at port " + self.port + ". Check to make sure your device is connected.")
            sys.exit(1)

    def read(self, register, numBytes=1):


