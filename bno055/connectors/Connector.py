from rclpy.node import Node
from bno055 import registers

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
        buf_out.append(registers.START_BYTE_WR)
        buf_out.append(registers.READ)
        buf_out.append(reg_addr)
        buf_out.append(length)

        try:
            self.write(buf_out)
            buf_in = bytearray(self.read(2 + length))
            # print("Reading, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
        except Exception:
            return 0

        # Check for valid response length (the smallest (error) message has at least 2 bytes):
        if buf_in.__len__() < 2:
            self.node.get_logger().warn("Unexpected length of READ-request response: %s" % buf_in.__len__())
            return 0

        # Check for READ result (success or failure):
        if buf_in[0] == registers.START_BYTE_ERROR_RESP:
            # Error 0x07 (BUS_OVER_RUN_ERROR) can be "normal" if data fusion is not yet ready- see also
            # https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-0x07-error-over-UART/td-p/14740
            self.node.get_logger().warn("READ-request failed with error code %s" % hex(buf_in[1]))
            return 0

        # Check for correct READ response header:
        if buf_in[0] != registers.START_BYTE_RESP:
            self.node.get_logger().warn("Wrong READ-request response header %s" % hex(buf_in[0]))
            return 0

        if (buf_in.__len__()-2) != buf_in[1]:
            self.node.get_logger().warn("Payload length mismatch detected : received=%s awaited=%s"
                                        % (buf_in.__len__()-2, buf_in[1]))
            return 0

        # Check for correct READ-request response length
        if buf_in.__len__() != (2 + length):
            self.node.get_logger().warn("Incorrect READ-request response length: %s" % (2 + length))
            return 0

        # remove the 0xBB:
        buf_in.pop(0)
        # remove the length information:
        buf_in.pop(0)

        # Return the received payload:
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
        buf_out.append(registers.START_BYTE_WR)
        buf_out.append(registers.WRITE)
        buf_out.append(reg_addr)
        buf_out.append(length)
        buf_out.append(data)

        try:
            self.write(buf_out)
            buf_in = bytearray(self.read(2))
            # print("Writing, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
        except Exception:
            return False

        if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
            # rospy.logerr("Incorrect Bosh IMU device response.")
            return False
        return True
