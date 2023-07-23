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


from bno055.connectors.uart import UART
from bno055.connectors.i2c import I2C
from bno055 import registers
from rclpy.node import Node


class NodeParameters:
    """
    ROS2 Node Parameter Handling.

    https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters
    https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-Python/

    Start the node with parameters from yml file:
    ros2 run bno055 bno055

    with the following arguments:
    --ros-args --params-file <workspace>/src/bno055/bno055/params/bno055_params.yaml
    """

    def __init__(self, node: Node):
        node.get_logger().info('Initializing parameters')
        # Declare parameters of the ROS2 node and their default values:

        # The topic prefix to use (can be empty if not required)
        node.declare_parameter(name='ros_topic_prefix', value='bno055/')
        # The type of the sensor connection. Either "uart" or "i2c":
        node.declare_parameter(name='connection_type', value=UART.CONNECTIONTYPE_UART)
        # I2C bus number
        node.declare_parameter('i2c_bus', value=0)
        # I2C address
        node.declare_parameter('i2c_addr', value=0x28)
        # UART port
        node.declare_parameter('uart_port', value='/dev/ttyUSB0')
        # UART Baud Rate
        node.declare_parameter('uart_baudrate', value=115200)
        # UART Timeout in seconds
        node.declare_parameter('uart_timeout', value=0.1)
        # tf frame id
        node.declare_parameter('frame_id', value='bno055')
        # Node timer frequency in Hz, defining how often sensor data is requested
        node.declare_parameter('data_query_frequency', value=10)
        # Node timer frequency in Hz, defining how often calibration status data is requested
        node.declare_parameter('calib_status_frequency', value=0.1)
        # sensor operation mode
        node.declare_parameter('operation_mode', value=0x0C)
        # placement_axis_remap defines the position and orientation of the sensor mount
        node.declare_parameter('placement_axis_remap', value='P1')
        # scaling factor for acceleration
        node.declare_parameter('acc_factor', value=100.0)
        # scaling factor for magnetometer
        node.declare_parameter('mag_factor', value=16000000.0)
        # scaling factor for gyroscope
        node.declare_parameter('gyr_factor', value=900.0)
        # scaling factor for gyroscope
        node.declare_parameter('grav_factor', value=100.0)
        # determines whether to use default offsets or not
        node.declare_parameter('set_offsets', value=False)
        # +/- 2000 units (at max 2G) (1 unit = 1 mg = 1 LSB = 0.01 m/s2)
        node.declare_parameter('offset_acc', value=registers.DEFAULT_OFFSET_ACC)
        # +/- 6400 units (1 unit = 1/16 uT)
        node.declare_parameter('offset_mag', value=registers.DEFAULT_OFFSET_MAG)
        # +/- 2000 units up to 32000 (dps range dependent)               (1 unit = 1/16 dps)
        node.declare_parameter('offset_gyr', value=registers.DEFAULT_OFFSET_GYR)
        # +/-1000 units
        node.declare_parameter('radius_acc', value=registers.DEFAULT_RADIUS_ACC)
        #  +/-960 units
        node.declare_parameter('radius_mag', value=registers.DEFAULT_RADIUS_MAG)
        # Sensor standard deviation squared (^2) defaults [x, y, z]
        node.declare_parameter('variance_acc', value=registers.DEFAULT_VARIANCE_ACC)
        node.declare_parameter('variance_angular_vel', value=registers.DEFAULT_VARIANCE_ANGULAR_VEL)
        node.declare_parameter('variance_orientation', value=registers.DEFAULT_VARIANCE_ORIENTATION)
        node.declare_parameter('variance_mag', value=registers.DEFAULT_VARIANCE_MAG)

        # get the parameters - requires CLI arguments '--ros-args --params-file <parameter file>'
        node.get_logger().info('Parameters set to:')

        try:
            self.ros_topic_prefix = node.get_parameter('ros_topic_prefix')
            node.get_logger().info('\tros_topic_prefix:\t"%s"' % self.ros_topic_prefix.value)

            self.connection_type = node.get_parameter('connection_type')
            node.get_logger().info('\tconnection_type:\t"%s"' % self.connection_type.value)

            if self.connection_type.value == I2C.CONNECTIONTYPE_I2C:

                self.i2c_bus = node.get_parameter('i2c_bus')
                node.get_logger().info('\ti2c_bus:\t\t"%s"' % self.i2c_bus.value)

                self.i2c_addr = node.get_parameter('i2c_addr')
                node.get_logger().info('\ti2c_addr:\t\t"%s"' % self.i2c_addr.value)

            elif self.connection_type.value == UART.CONNECTIONTYPE_UART:

                self.uart_port = node.get_parameter('uart_port')
                node.get_logger().info('\tuart_port:\t\t"%s"' % self.uart_port.value)

                self.uart_baudrate = node.get_parameter('uart_baudrate')
                node.get_logger().info('\tuart_baudrate:\t\t"%s"' % self.uart_baudrate.value)

                self.uart_timeout = node.get_parameter('uart_timeout')
                node.get_logger().info('\tuart_timeout:\t\t"%s"' % self.uart_timeout.value)

            self.frame_id = node.get_parameter('frame_id')
            node.get_logger().info('\tframe_id:\t\t"%s"' % self.frame_id.value)

            self.data_query_frequency = node.get_parameter('data_query_frequency')
            node.get_logger().info('\tdata_query_frequency:\t"%s"'
                                   % self.data_query_frequency.value)

            self.calib_status_frequency = node.get_parameter('calib_status_frequency')
            node.get_logger().info('\tcalib_status_frequency:\t"%s"'
                                   % self.calib_status_frequency.value)

            self.operation_mode = node.get_parameter('operation_mode')
            node.get_logger().info('\toperation_mode:\t\t"%s"' % self.operation_mode.value)

            self.placement_axis_remap = node.get_parameter('placement_axis_remap')
            node.get_logger().info('\tplacement_axis_remap:\t"%s"'
                                   % self.placement_axis_remap.value)

            self.acc_factor = node.get_parameter('acc_factor')
            node.get_logger().info('\tacc_factor:\t\t"%s"' % self.acc_factor.value)

            self.mag_factor = node.get_parameter('mag_factor')
            node.get_logger().info('\tmag_factor:\t\t"%s"' % self.mag_factor.value)

            self.gyr_factor = node.get_parameter('gyr_factor')
            node.get_logger().info('\tgyr_factor:\t\t"%s"' % self.gyr_factor.value)

            self.grav_factor = node.get_parameter('grav_factor')
            node.get_logger().info('\tgrav_factor:\t\t"%s"' % self.grav_factor.value)

            self.set_offsets = node.get_parameter('set_offsets')
            node.get_logger().info('\tset_offsets:\t\t"%s"' % self.set_offsets.value)

            self.offset_acc = node.get_parameter('offset_acc')
            node.get_logger().info('\toffset_acc:\t\t"%s"' % self.offset_acc.value)

            self.radius_acc = node.get_parameter('radius_acc')
            node.get_logger().info('\tradius_acc:\t\t"%s"' % self.radius_acc.value)

            self.offset_mag = node.get_parameter('offset_mag')
            node.get_logger().info('\toffset_mag:\t\t"%s"' % self.offset_mag.value)

            self.radius_mag = node.get_parameter('radius_mag')
            node.get_logger().info('\tradius_mag:\t\t"%s"' % self.radius_mag.value)

            self.offset_gyr = node.get_parameter('offset_gyr')
            node.get_logger().info('\toffset_gyr:\t\t"%s"' % self.offset_gyr.value)

            self.variance_acc = node.get_parameter('variance_acc')
            node.get_logger().info('\tvariance_acc:\t\t"%s"' % self.variance_acc.value)
            self.variance_angular_vel = node.get_parameter('variance_angular_vel')
            node.get_logger().info('\tvariance_angular_vel:\t"%s"' % self.variance_angular_vel.value)
            self.variance_orientation = node.get_parameter('variance_orientation')
            node.get_logger().info('\tvariance_orientation:\t"%s"' % self.variance_orientation.value)
            self.variance_mag = node.get_parameter('variance_mag')
            node.get_logger().info('\tvariance_mag:\t\t"%s"' % self.variance_mag.value)
        except Exception as e:  # noqa: B902
            node.get_logger().warn('Could not get parameters...setting variables to default')
            node.get_logger().warn('Error: "%s"' % e)
