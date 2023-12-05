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
from abc import ABC, abstractclassmethod
from enum import Enum
from pydantic import BaseModel, ConfigDict

from rclpy.node import Node


class ConnectorType(Enum):
    I2C = "i2c"
    UART = "uart"


class Connector(BaseModel, ABC):
    """
    Parent class for bno055 connectors.

    This class does NOT contain protocol-specific code for UART, I2C, etc.
    """
    # To allow for Node type attribute
    model_config = ConfigDict(arbitrary_types_allowed=True)

    node: Node
    type: ConnectorType

    @property
    @abstractclassmethod
    def read(self, reg_addr, length: int) -> bytearray:
        """
        Read the given length of memory starting at the given register address.

        Required to be overridden by inheriting classes of this base class.

        :param reg_addr: The register address
        :param length: The data length
        :return: bytearray of the read data
        """
        raise NotImplementedError()

    def receive(self, reg_addr, length: int) -> bytearray:
        return self.read(reg_addr, length)

    def transmit(self, reg_addr, length: int, data: bytes):
        return self.write(reg_addr, length, data)
