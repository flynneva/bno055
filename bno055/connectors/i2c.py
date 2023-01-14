# Copyright 2021 AUTHORS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AUTHORS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from smbus import SMBus

from rclpy.node import Node

from bno055 import registers
from bno055.connectors.Connector import Connector
from bno055.error_handling.exceptions import TransmissionException


class I2C(Connector):
    """Connector implementation for I2C connection to the sensor."""

    CONNECTIONTYPE_I2C = 'i2c'

    def __init__(self, node: Node, i2c_bus=0, i2c_addr=registers.BNO055_ADDRESS_A):
        """Initialize the I2C class.
        
        :param node: a ROS node
        :param i2c_bus: I2C bus to use
        :param i2c_addr: I2C address to connect to
        :return:
        """
        super().__init__(node)
        self.bus = SMBus(i2c_bus)
        self.address = i2c_addr

    def connect(self):
        """Connect to the sensor
        
        :return:
        """
        returned_id = self.bus.read_byte_data(self.address, registers.BNO055_CHIP_ID_ADDR)
        if returned_id != registers.BNO055_ID:
            raise TransmissionException('Could not get BNO055 chip ID via I2C')

    def read(self, reg_addr, length):
        """Read data from sensor via I2C.

        :param reg_addr: The register address
        :param length: The data length
        :return:
        """
        buffer = bytearray()
        bytes_left_to_read = length
        while bytes_left_to_read > 0:
            read_len = min(bytes_left_to_read, 32)
            read_off = length - bytes_left_to_read
            response = self.bus.read_i2c_block_data(
                self.address, reg_addr + read_off, read_len)
            buffer += bytearray(response)
            bytes_left_to_read -= read_len
        return buffer

    def write(self, reg_addr, length, data: bytes):
        """Write data to sensor via I2C.
        
        :param reg_addr: The register address
        :param length: The data length
        :param data: data to transmit
        :return:
        """
        bytes_left_to_write = length
        while bytes_left_to_write > 0:
            write_len = min(bytes_left_to_write, 32)
            write_off = length - bytes_left_to_write
            datablock = list(data[write_off : write_off + write_len])
            self.bus.write_i2c_block_data(
                self.address, reg_addr + write_off, datablock)
            bytes_left_to_write -= write_len
        return True
