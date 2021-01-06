from rclpy.node import Node
from bno055.connectors.uart import UART


class NodeParameters:
    """
    ROS2 Node Parameter Handling
    See also

    https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters

    https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-Python/

    Start the node with parameters from yml file:
    ros2 run bno055 bno055 --ros-args --params-file <workspace>/src/bno055/bno055/params/bno055_params.yaml
    """

    def __init__(self, node: Node):

        node.get_logger().info('Initializing parameters')

        # Declare parameters of the ROS2 node and their default values:

        # TODO add I2C configuration parameters

        # The type of the sensor connection. Either "uart" or "i2c":
        node.declare_parameter(name='connection_type', value=UART.CONNECTIONTYPE_UART)
        # UART port
        node.declare_parameter('port', value='/dev/ttyUSB0')
        # UART Baud Rate
        node.declare_parameter('baudrate', value=115200)
        # tf frame id
        node.declare_parameter('frame_id', value='bno055')
        # Node timer frequency in Hz
        node.declare_parameter('frequency', value=100)
        # sensor operation mode
        node.declare_parameter('operation_mode', value=0x0C)
        # +/- 2000 units (at max 2G) (1 unit = 1 mg = 1 LSB = 0.01 m/s2)
        node.declare_parameter('acc_offset', value=[0xFFEC, 0x00A5, 0xFFE8])
        # +/- 6400 units (1 unit = 1/16 uT)
        node.declare_parameter('mag_offset', value=[0xFFB4, 0xFE9E, 0x027D])
        # +/- 2000 units up to 32000 (dps range dependent)               (1 unit = 1/16 dps)
        node.declare_parameter('gyr_offset', value=[0x0002, 0xFFFF, 0xFFFF])

        # get the parameters - requires CLI arguments '--ros-args --params-file <parameter file>'
        node.get_logger().info('Parameters set to:')

        try:
            self.connection_type = node.get_parameter('connection_type')
            node.get_logger().info('\tconnection_type:\t"%s"' % self.connection_type.value)

            self.port = node.get_parameter('port')
            node.get_logger().info('\tport:\t\t\t"%s"' % self.port.value)

            self.baudrate = node.get_parameter('baudrate')
            node.get_logger().info('\tbaudrate:\t\t"%s"' % self.baudrate.value)

            self.frame_id = node.get_parameter('frame_id')
            node.get_logger().info('\tframe_id:\t\t"%s"' % self.frame_id.value)

            self.frequency = node.get_parameter('frequency')
            node.get_logger().info('\tfrequency:\t\t"%s"' % self.frequency.value)

            self.operation_mode = node.get_parameter('operation_mode')
            node.get_logger().info('\toperation_mode:\t\t"%s"' % self.operation_mode.value)

            self.acc_offset = node.get_parameter('acc_offset')
            node.get_logger().info('\tacc_offset:\t\t"%s"' % self.acc_offset.value)

            self.mag_offset = node.get_parameter('mag_offset')
            node.get_logger().info('\tmag_offset:\t\t"%s"' % self.mag_offset.value)

            self.gyr_offset = node.get_parameter('gyr_offset')
            node.get_logger().info('\tgyr_offset:\t\t"%s"' % self.gyr_offset.value)

        except Exception as e:
            node.get_logger().warn('Could not get parameters...setting variables to default')
            node.get_logger().warn('Error: "%s"' % e)

