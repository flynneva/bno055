import sys
import threading

from bno055.connectors.i2c import I2C
from bno055.connectors.uart import UART
from bno055.params.NodeParameters import NodeParameters
from bno055.sensor.SensorService import SensorService
import rclpy
from rclpy.node import Node


class Bno055Node(Node):
    """ROS2 Node for interfacing Bosch Bno055 IMU sensor."""

    sensor = None
    param = None

    def __init__(self):
        # Initialize parent (ROS Node)
        super().__init__('bno055')

    def setup(self):
        # Initialize ROS2 Node Parameters:
        self.param = NodeParameters(self)

        # Get connector according to configured sensor connection type:
        if self.param.connection_type.value == UART.CONNECTIONTYPE_UART:
            connector = UART(self,
                             self.param.uart_baudrate.value,
                             self.param.uart_port.value,
                             self.param.uart_timeout.value)
        elif self.param.connection_type.value == I2C.CONNECTIONTYPE_I2C:
            # TODO implement IC2 integration
            raise NotImplementedError('I2C not yet implemented')
        else:
            raise NotImplementedError('Unsupported connection type: '
                                      + str(self.param.connection_type.value))

        # Connect to BNO055 device:
        connector.connect()

        # Instantiate the sensor Service API:
        self.sensor = SensorService(self, connector, self.param)

        # configure imu
        self.sensor.configure()


def main(args=None):
    try:
        """Main entry method for this ROS2 node."""
        # Initialize ROS Client Libraries (RCL) for Python:
        rclpy.init()

        # Create & initialize ROS2 node:
        node = Bno055Node()
        node.setup()

        # Create lock object to prevent overlapping data queries
        lock = threading.Lock()

        def read_data():
            """Callback for periodic data_query_timer executions to retrieve sensor IMU data."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_sensor_data()
            except Exception as e:
                node.get_logger().warn('Receiving sensor data failed with %s:"%s"'
                                       % (type(e).__name__, e))
            finally:
                lock.release()

        def log_calibration_status():
            """Callback for periodic logging of calibration data (quality indicators)."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn('Message communication in progress - skipping query cycle')
                # traceback.print_exc()
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_calib_status()
            except Exception as e:
                node.get_logger().warn('Receiving calibration status failed with %s:"%s"'
                                       % (type(e).__name__, e))
                # traceback.print_exc()
            finally:
                lock.release()

        # start regular sensor transmissions:
        # please be aware that frequencies around 30Hz and above might cause performance impacts:
        # https://github.com/ros2/rclpy/issues/520
        f = 1.0 / float(node.param.data_query_frequency.value)
        data_query_timer = node.create_timer(f, read_data)

        # start regular calibration status logging
        f = 1.0 / float(node.param.calib_status_frequency.value)
        status_timer = node.create_timer(f, log_calibration_status)

        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - exiting...')
        sys.exit(0)
    finally:
        node.get_logger().info('ROS node shutdown')
        node.destroy_timer(data_query_timer)
        node.destroy_timer(status_timer)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
