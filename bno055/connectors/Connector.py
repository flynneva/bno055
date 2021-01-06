from rclpy.node import Node
from bno055 import bno055_registers

import binascii

class Connector:
    """
    Parent class for bno055 connectors.
    This class does NOT contain protocol-specific code for UART, I2C, etc.
    """

    def __init__(self, node: Node):
        self.node = node

    def receive(self, reg_addr, length):
        """
        Receives data packages from the sensor
        :param reg_addr: The register address
        :param length: The length of the data package to receive
        :return:
        """
        buf_out = bytearray()
        buf_out.append(bno055_registers.START_BYTE_WR)
        buf_out.append(bno055_registers.READ)
        buf_out.append(reg_addr)
        buf_out.append(length)

        try:
            self.write(buf_out)
            buf_in = bytearray(self.read(2 + length))
            #print("Reading, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
        except:
            return 0

        # Check if response is correct
        if (buf_in.__len__() != (2 + length)) or (buf_in[0] != bno055_registers.START_BYTE_RESP):
            #node.get_logger().warn("Incorrect device response.")
            return 0
        buf_in.pop(0)
        buf_in.pop(0)
        return buf_in

    # -----------------------------
    def transmit(self, reg_addr, length, data):
        """
        Transmit data packages to the sensor
        :param reg_addr: The register address
        :param length: The data length
        :param data: data to transmit
        :return:
        """
        buf_out = bytearray()
        buf_out.append(bno055_registers.START_BYTE_WR)
        buf_out.append(bno055_registers.WRITE)
        buf_out.append(reg_addr)
        buf_out.append(length)
        buf_out.append(data)

        try:
            usb_con.write(buf_out)
            buf_in = bytearray(usb_con.read(2))
            # print("Writing, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
        except:
            return False

        if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
            #rospy.logerr("Incorrect Bosh IMU device response.")
            return False
        return True


