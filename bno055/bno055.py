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

import sys
import threading

import rclpy
from rclpy.node import Node

from bno055.connectors.i2c import I2C
from bno055.connectors.uart import UART
from bno055.error_handling.exceptions import BusOverRunException
from bno055.params.NodeParameters import NodeParameters
from bno055.sensor.SensorService import SensorService


class Bno055Node(Node):
    """ROS2 Node for interfacing Bosch BNO055 IMU sensor."""

    def __init__(self):
        super().__init__('bno055')

        # Instance attributes
        self.param = None
        self.sensor = None

        # Timers + lock on the node
        self._lock = threading.Lock()
        self._data_timer = None
        self._status_timer = None

    def setup(self) -> None:
        """Configure params, connector, sensor service, and start timers."""
        self.param = NodeParameters(self)

        # Connector selection
        if self.param.connection_type.value == UART.CONNECTIONTYPE_UART:
            connector = UART(
                self,
                self.param.uart_baudrate.value,
                self.param.uart_port.value,
                self.param.uart_timeout.value,
            )
        elif self.param.connection_type.value == I2C.CONNECTIONTYPE_I2C:
            connector = I2C(
                self,
                self.param.i2c_bus.value,   # INTEGER, e.g. 1 for Raspberry Pi
                self.param.i2c_addr.value,  # INTEGER_ARRAY, e.g. [0x28, 0x29] for Adafruit BNO055 and GY-85 clone with both jumpers closed
            )
        else:
            raise NotImplementedError(
                f"Unsupported connection type: {self.param.connection_type.value}"
            )

        # Connect to BNO055 device:
        connector.connect()

        # Instantiate the sensor Service API:
        self.sensor = SensorService(self, connector, self.param)

        # configure imu
        self.sensor.configure()

        self._start_timers()

    def _start_timers(self) -> None:
        """Create timers that periodically query IMU + calibration status."""
        data_period = 1.0 / float(self.param.data_query_frequency.value)
        status_period = 1.0 / float(self.param.calib_status_frequency.value)

        self._data_timer = self.create_timer(data_period, self._read_data_cb)

        if self.sensor.is_fusing_mode:
            # Only start calib status timer if we are in a fusing mode that provides calibration status data.
            self._status_timer = self.create_timer(status_period, self._calib_status_cb)

        self.get_logger().info(
            f"Timers started: data={1.0/data_period:.2f} Hz, status={1.0/status_period:.2f} Hz"
        )

    def _is_shutting_down(self) -> bool:
        """
        True if ROS is shutting down / context is not OK.
        This is safe to call from timer callbacks.
        """
        return (not rclpy.ok()) or (not self.context.ok())

    def _read_data_cb(self) -> None:
        """Timer callback: read and publish IMU data."""
        if self._is_shutting_down():
            return

        if not self._lock.acquire(blocking=False):
            # Consider throttling this to avoid spam
            self.get_logger().warn("I/O busy - skipping IMU read cycle")
            return

        try:
            self.sensor.get_sensor_data()
        except BusOverRunException:
            # Usually means "data not ready"; warning every cycle can spam logs
            self.get_logger().debug("BusOverRunException (data not ready)")
        except ZeroDivisionError:
            self.get_logger().warn("ZeroDivisionError in get_sensor_data()")
        except Exception as e:  # noqa: B902
            self.get_logger().warn(f"Receiving sensor data failed with {type(e).__name__}: {e}")
        finally:
            self._lock.release()

    def _calib_status_cb(self) -> None:
        """Timer callback: read and publish calibration status."""
        if self._is_shutting_down():
            return

        if not self._lock.acquire(blocking=False):
            self.get_logger().warn("I/O busy - skipping calib status cycle")
            return

        try:
            self.sensor.get_calib_status()
        except Exception as e:  # noqa: B902
            self.get_logger().warn(f"Receiving calib status failed with {type(e).__name__}: {e}")
        finally:
            self._lock.release()

    def destroy_node(self):
        """
        Ensure timers are stopped before node teardown.
        (rclpy usually handles this, but explicit is safer.)
        """
        try:
            if self._data_timer is not None:
                self._data_timer.cancel()
                self.destroy_timer(self._data_timer)
                self._data_timer = None

            if self._status_timer is not None:
                self._status_timer.cancel()
                self.destroy_timer(self._status_timer)
                self._status_timer = None
        except Exception:
            pass

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Bno055Node()

    try:
        node.setup()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received - exiting...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
