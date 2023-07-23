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
import json
from math import sqrt
import struct
import sys
from time import sleep

from bno055 import registers
from bno055.connectors.Connector import Connector
from bno055.params.NodeParameters import NodeParameters

from geometry_msgs.msg import Quaternion, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import String
from example_interfaces.srv import Trigger


class SensorService:
    """Provide an interface for accessing the sensor's features & data."""

    def __init__(self, node: Node, connector: Connector, param: NodeParameters):
        self.node = node
        self.con = connector
        self.param = param

        prefix = self.param.ros_topic_prefix.value
        QoSProf = QoSProfile(depth=10)

        # create topic publishers:
        self.pub_imu_raw = node.create_publisher(Imu, prefix + 'imu_raw', QoSProf)
        self.pub_imu = node.create_publisher(Imu, prefix + 'imu', QoSProf)
        self.pub_mag = node.create_publisher(MagneticField, prefix + 'mag', QoSProf)
        self.pub_grav = node.create_publisher(Vector3, prefix + 'grav', QoSProf)
        self.pub_temp = node.create_publisher(Temperature, prefix + 'temp', QoSProf)
        self.pub_calib_status = node.create_publisher(String, prefix + 'calib_status', QoSProf)
        self.srv = self.node.create_service(Trigger, prefix + 'calibration_request', self.calibration_request_callback)

    def configure(self):
        """Configure the IMU sensor hardware."""
        self.node.get_logger().info('Configuring device...')
        try:
            data = self.con.receive(registers.BNO055_CHIP_ID_ADDR, 1)
            if data[0] != registers.BNO055_ID:
                raise IOError('Device ID=%s is incorrect' % data)
            # print("device sent ", binascii.hexlify(data))
        except Exception as e:  # noqa: B902
            # This is the first communication - exit if it does not work
            self.node.get_logger().error('Communication error: %s' % e)
            self.node.get_logger().error('Shutting down ROS node...')
            sys.exit(1)

        # IMU connected => apply IMU Configuration:
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            self.node.get_logger().warn('Unable to set IMU into config mode.')

        if not (self.con.transmit(registers.BNO055_PWR_MODE_ADDR, 1, bytes([registers.POWER_MODE_NORMAL]))):
            self.node.get_logger().warn('Unable to set IMU normal power mode.')

        if not (self.con.transmit(registers.BNO055_PAGE_ID_ADDR, 1, bytes([0x00]))):
            self.node.get_logger().warn('Unable to set IMU register page 0.')

        if not (self.con.transmit(registers.BNO055_SYS_TRIGGER_ADDR, 1, bytes([0x00]))):
            self.node.get_logger().warn('Unable to start IMU.')

        if not (self.con.transmit(registers.BNO055_UNIT_SEL_ADDR, 1, bytes([0x83]))):
            self.node.get_logger().warn('Unable to set IMU units.')

        # The sensor placement configuration (Axis remapping) defines the
        # position and orientation of the sensor mount.
        # See also Bosch BNO055 datasheet section Axis Remap
        mount_positions = {
            'P0': bytes(b'\x21\x04'),
            'P1': bytes(b'\x24\x00'),
            'P2': bytes(b'\x24\x06'),
            'P3': bytes(b'\x21\x02'),
            'P4': bytes(b'\x24\x03'),
            'P5': bytes(b'\x21\x02'),
            'P6': bytes(b'\x21\x07'),
            'P7': bytes(b'\x24\x05')
        }
        if not (self.con.transmit(registers.BNO055_AXIS_MAP_CONFIG_ADDR, 2,
                                  mount_positions[self.param.placement_axis_remap.value])):
            self.node.get_logger().warn('Unable to set sensor placement configuration.')

        # Show the current sensor offsets
        self.node.get_logger().info('Current sensor offsets:')
        self.print_calib_data()
        if self.param.set_offsets.value:
            configured_offsets = \
                self.set_calib_offsets(
                    self.param.offset_acc,
                    self.param.offset_mag,
                    self.param.offset_gyr,
                    self.param.radius_mag,
                    self.param.radius_acc)
            if configured_offsets:
                self.node.get_logger().info('Successfully configured sensor offsets to:')
                self.print_calib_data()
            else:
                self.node.get_logger().warn('setting offsets failed')


        # Set Device mode
        device_mode = self.param.operation_mode.value
        self.node.get_logger().info(f"Setting device_mode to {device_mode}")

        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([device_mode]))):
            self.node.get_logger().warn('Unable to set IMU operation mode into operation mode.')

        self.node.get_logger().info('Bosch BNO055 IMU configuration complete.')

    def get_sensor_data(self):
        """Read IMU data from the sensor, parse and publish."""
        # Initialize ROS msgs
        imu_raw_msg = Imu()
        imu_msg = Imu()
        mag_msg = MagneticField()
        grav_msg = Vector3()
        temp_msg = Temperature()

        # read from sensor
        buf = self.con.receive(registers.BNO055_ACCEL_DATA_X_LSB_ADDR, 45)
        # Publish raw data
        imu_raw_msg.header.stamp = self.node.get_clock().now().to_msg()
        imu_raw_msg.header.frame_id = self.param.frame_id.value
        # TODO: do headers need sequence counters now?
        # imu_raw_msg.header.seq = seq

        # TODO: make this an option to publish?
        imu_raw_msg.orientation_covariance = [
            self.param.variance_orientation.value[0], 0.0, 0.0,
            0.0, self.param.variance_orientation.value[1], 0.0,
            0.0, 0.0, self.param.variance_orientation.value[2]
        ]

        imu_raw_msg.linear_acceleration.x = \
            self.unpackBytesToFloat(buf[0], buf[1]) / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration.y = \
            self.unpackBytesToFloat(buf[2], buf[3]) / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration.z = \
            self.unpackBytesToFloat(buf[4], buf[5]) / self.param.acc_factor.value
        imu_raw_msg.linear_acceleration_covariance = [
            self.param.variance_acc.value[0], 0.0, 0.0,
            0.0, self.param.variance_acc.value[1], 0.0,
            0.0, 0.0, self.param.variance_acc.value[2]
        ]
        imu_raw_msg.angular_velocity.x = \
            self.unpackBytesToFloat(buf[12], buf[13]) / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity.y = \
            self.unpackBytesToFloat(buf[14], buf[15]) / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity.z = \
            self.unpackBytesToFloat(buf[16], buf[17]) / self.param.gyr_factor.value
        imu_raw_msg.angular_velocity_covariance = [
            self.param.variance_angular_vel.value[0], 0.0, 0.0,
            0.0, self.param.variance_angular_vel.value[1], 0.0,
            0.0, 0.0, self.param.variance_angular_vel.value[2]
        ]
        # node.get_logger().info('Publishing imu message')
        self.pub_imu_raw.publish(imu_raw_msg)

        # TODO: make this an option to publish?
        # Publish filtered data
        imu_msg.header.stamp = self.node.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.param.frame_id.value

        q = Quaternion()
        # imu_msg.header.seq = seq
        q.w = self.unpackBytesToFloat(buf[24], buf[25])
        q.x = self.unpackBytesToFloat(buf[26], buf[27])
        q.y = self.unpackBytesToFloat(buf[28], buf[29])
        q.z = self.unpackBytesToFloat(buf[30], buf[31])
        # TODO(flynneva): replace with standard normalize() function
        # normalize
        norm = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        imu_msg.orientation.x = q.x / norm
        imu_msg.orientation.y = q.y / norm
        imu_msg.orientation.z = q.z / norm
        imu_msg.orientation.w = q.w / norm

        imu_msg.orientation_covariance = imu_raw_msg.orientation_covariance

        imu_msg.linear_acceleration.x = \
            self.unpackBytesToFloat(buf[32], buf[33]) / self.param.acc_factor.value
        imu_msg.linear_acceleration.y = \
            self.unpackBytesToFloat(buf[34], buf[35]) / self.param.acc_factor.value
        imu_msg.linear_acceleration.z = \
            self.unpackBytesToFloat(buf[36], buf[37]) / self.param.acc_factor.value
        imu_msg.linear_acceleration_covariance = imu_raw_msg.linear_acceleration_covariance
        imu_msg.angular_velocity.x = \
            self.unpackBytesToFloat(buf[12], buf[13]) / self.param.gyr_factor.value
        imu_msg.angular_velocity.y = \
            self.unpackBytesToFloat(buf[14], buf[15]) / self.param.gyr_factor.value
        imu_msg.angular_velocity.z = \
            self.unpackBytesToFloat(buf[16], buf[17]) / self.param.gyr_factor.value
        imu_msg.angular_velocity_covariance = imu_raw_msg.angular_velocity_covariance
        self.pub_imu.publish(imu_msg)

        # Publish magnetometer data
        mag_msg.header.stamp = self.node.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.param.frame_id.value
        # mag_msg.header.seq = seq
        mag_msg.magnetic_field.x = \
            self.unpackBytesToFloat(buf[6], buf[7]) / self.param.mag_factor.value
        mag_msg.magnetic_field.y = \
            self.unpackBytesToFloat(buf[8], buf[9]) / self.param.mag_factor.value
        mag_msg.magnetic_field.z = \
            self.unpackBytesToFloat(buf[10], buf[11]) / self.param.mag_factor.value
        mag_msg.magnetic_field_covariance = [
            self.param.variance_mag.value[0], 0.0, 0.0,
            0.0, self.param.variance_mag.value[1], 0.0,
            0.0, 0.0, self.param.variance_mag.value[2]
        ]
        self.pub_mag.publish(mag_msg)

        grav_msg.x = \
            self.unpackBytesToFloat(buf[38], buf[39]) / self.param.grav_factor.value
        grav_msg.y = \
            self.unpackBytesToFloat(buf[40], buf[41]) / self.param.grav_factor.value
        grav_msg.z = \
            self.unpackBytesToFloat(buf[42], buf[43]) / self.param.grav_factor.value
        self.pub_grav.publish(grav_msg)

        # Publish temperature
        temp_msg.header.stamp = self.node.get_clock().now().to_msg()
        temp_msg.header.frame_id = self.param.frame_id.value
        # temp_msg.header.seq = seq
        temp_msg.temperature = float(buf[44])
        self.pub_temp.publish(temp_msg)

    def get_calib_status(self):
        """
        Read calibration status for sys/gyro/acc/mag.

        Quality scale: 0 = bad, 3 = best
        """
        calib_status = self.con.receive(registers.BNO055_CALIB_STAT_ADDR, 1)
        sys = (calib_status[0] >> 6) & 0x03
        gyro = (calib_status[0] >> 4) & 0x03
        accel = (calib_status[0] >> 2) & 0x03
        mag = calib_status[0] & 0x03

        # Create dictionary (map) and convert it to JSON string:
        calib_status_dict = {'sys': sys, 'gyro': gyro, 'accel': accel, 'mag': mag}
        calib_status_str = String()
        calib_status_str.data = json.dumps(calib_status_dict)

        # Publish via ROS topic:
        self.pub_calib_status.publish(calib_status_str)

    def get_calib_data(self):
        """Read all calibration data."""

        accel_offset_read = self.con.receive(registers.ACCEL_OFFSET_X_LSB_ADDR, 6)
        accel_offset_read_x = (accel_offset_read[1] << 8) | accel_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        accel_offset_read_y = (accel_offset_read[3] << 8) | accel_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        accel_offset_read_z = (accel_offset_read[5] << 8) | accel_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        accel_radius_read = self.con.receive(registers.ACCEL_RADIUS_LSB_ADDR, 2)
        accel_radius_read_value = (accel_radius_read[1] << 8) | accel_radius_read[0]

        mag_offset_read = self.con.receive(registers.MAG_OFFSET_X_LSB_ADDR, 6)
        mag_offset_read_x = (mag_offset_read[1] << 8) | mag_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_y = (mag_offset_read[3] << 8) | mag_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        mag_offset_read_z = (mag_offset_read[5] << 8) | mag_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        mag_radius_read = self.con.receive(registers.MAG_RADIUS_LSB_ADDR, 2)
        mag_radius_read_value = (mag_radius_read[1] << 8) | mag_radius_read[0]

        gyro_offset_read = self.con.receive(registers.GYRO_OFFSET_X_LSB_ADDR, 6)
        gyro_offset_read_x = (gyro_offset_read[1] << 8) | gyro_offset_read[
            0]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_y = (gyro_offset_read[3] << 8) | gyro_offset_read[
            2]  # Combine MSB and LSB registers into one decimal
        gyro_offset_read_z = (gyro_offset_read[5] << 8) | gyro_offset_read[
            4]  # Combine MSB and LSB registers into one decimal

        calib_data = {'accel_offset': {'x': accel_offset_read_x, 'y': accel_offset_read_y, 'z': accel_offset_read_z}, 'accel_radius': accel_radius_read_value,
                      'mag_offset': {'x': mag_offset_read_x, 'y': mag_offset_read_y, 'z': mag_offset_read_z}, 'mag_radius': mag_radius_read_value,
                      'gyro_offset': {'x': gyro_offset_read_x, 'y': gyro_offset_read_y, 'z': gyro_offset_read_z}}

        return calib_data

    def print_calib_data(self):
        """Read all calibration data and print to screen."""
        calib_data = self.get_calib_data()
        self.node.get_logger().info(
            '\tAccel offsets (x y z): %d %d %d' % (
                calib_data['accel_offset']['x'],
                calib_data['accel_offset']['y'],
                calib_data['accel_offset']['z']))

        self.node.get_logger().info(
            '\tAccel radius: %d' % (
                calib_data['accel_radius'],
            )
        )

        self.node.get_logger().info(
            '\tMag offsets (x y z): %d %d %d' % (
                calib_data['mag_offset']['x'],
                calib_data['mag_offset']['y'],
                calib_data['mag_offset']['z']))

        self.node.get_logger().info(
            '\tMag radius: %d' % (
                calib_data['mag_radius'],
            )
        )

        self.node.get_logger().info(
            '\tGyro offsets (x y z): %d %d %d' % (
                calib_data['gyro_offset']['x'],
                calib_data['gyro_offset']['y'],
                calib_data['gyro_offset']['z']))

    def set_calib_offsets(self, acc_offset, mag_offset, gyr_offset, mag_radius, acc_radius):
        """
        Write calibration data (define as 16 bit signed hex).

        :param acc_offset:
        :param mag_offset:
        :param gyr_offset:
        :param mag_radius:
        :param acc_radius:
        """
        # Must switch to config mode to write out
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            self.node.get_logger().error('Unable to set IMU into config mode')
        sleep(0.025)

        # Seems to only work when writing 1 register at a time
        try:
            self.con.transmit(registers.ACCEL_OFFSET_X_LSB_ADDR, 1, bytes([acc_offset.value[0] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_X_MSB_ADDR, 1, bytes([(acc_offset.value[0] >> 8) & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Y_LSB_ADDR, 1, bytes([acc_offset.value[1] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Y_MSB_ADDR, 1, bytes([(acc_offset.value[1] >> 8) & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Z_LSB_ADDR, 1, bytes([acc_offset.value[2] & 0xFF]))
            self.con.transmit(registers.ACCEL_OFFSET_Z_MSB_ADDR, 1, bytes([(acc_offset.value[2] >> 8) & 0xFF]))

            self.con.transmit(registers.ACCEL_RADIUS_LSB_ADDR, 1, bytes([acc_radius.value & 0xFF]))
            self.con.transmit(registers.ACCEL_RADIUS_MSB_ADDR, 1, bytes([(acc_radius.value >> 8) & 0xFF]))

            self.con.transmit(registers.MAG_OFFSET_X_LSB_ADDR, 1, bytes([mag_offset.value[0] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_X_MSB_ADDR, 1, bytes([(mag_offset.value[0] >> 8) & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Y_LSB_ADDR, 1, bytes([mag_offset.value[1] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Y_MSB_ADDR, 1, bytes([(mag_offset.value[1] >> 8) & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Z_LSB_ADDR, 1, bytes([mag_offset.value[2] & 0xFF]))
            self.con.transmit(registers.MAG_OFFSET_Z_MSB_ADDR, 1, bytes([(mag_offset.value[2] >> 8) & 0xFF]))

            self.con.transmit(registers.MAG_RADIUS_LSB_ADDR, 1, bytes([mag_radius.value & 0xFF]))
            self.con.transmit(registers.MAG_RADIUS_MSB_ADDR, 1, bytes([(mag_radius.value >> 8) & 0xFF]))

            self.con.transmit(registers.GYRO_OFFSET_X_LSB_ADDR, 1, bytes([gyr_offset.value[0] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_X_MSB_ADDR, 1, bytes([(gyr_offset.value[0] >> 8) & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Y_LSB_ADDR, 1, bytes([gyr_offset.value[1] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Y_MSB_ADDR, 1, bytes([(gyr_offset.value[1] >> 8) & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Z_LSB_ADDR, 1, bytes([gyr_offset.value[2] & 0xFF]))
            self.con.transmit(registers.GYRO_OFFSET_Z_MSB_ADDR, 1, bytes([(gyr_offset.value[2] >> 8) & 0xFF]))

            return True
        except Exception:  # noqa: B902
            return False

    def calibration_request_callback(self, request, response):
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_CONFIG]))):
            self.node.get_logger().warn('Unable to set IMU into config mode.')
        sleep(0.025)
        calib_data = self.get_calib_data()
        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([registers.OPERATION_MODE_NDOF]))):
            self.node.get_logger().warn('Unable to set IMU operation mode into operation mode.')
        response.success = True
        response.message = str(calib_data)
        return response

    def unpackBytesToFloat(self, start, end):
        return float(struct.unpack('h', struct.pack('BB', start, end))[0])
