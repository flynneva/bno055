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
import struct
import sys
from time import sleep
import numpy as np

from bno055 import registers
from bno055.connectors.Connector import Connector
from bno055.params.NodeParameters import NodeParameters

from geometry_msgs.msg import Vector3   # or Vector3Stamped
import rclpy
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
        qos_profile = QoSProfile(depth=10)

        # create topic publishers:
        self.pub_imu_raw = node.create_publisher(Imu, prefix + 'imu_raw', qos_profile)
        self.pub_imu = node.create_publisher(Imu, prefix + 'imu', qos_profile)
        self.pub_mag = node.create_publisher(MagneticField, prefix + 'mag', qos_profile)
        self.pub_grav = node.create_publisher(Vector3, prefix + 'grav', qos_profile)   # or Vector3Stamped
        self.pub_temp = node.create_publisher(Temperature, prefix + 'temp', qos_profile)
        self.pub_calib_status = node.create_publisher(String, prefix + 'calib_status', qos_profile)
        self.srv = self.node.create_service(Trigger, prefix + 'calibration_request', self.calibration_request_callback)

        # initialize message objects to reuse for publishing (avoid creating new objects every time):
        self._imu_raw_msg = Imu()
        self._imu_msg = Imu()
        self._mag_msg = MagneticField()
        self._grav_msg = Vector3()   # or Vector3Stamped
        self._temp_msg = Temperature()

        # precompute covariance matrices from parameters (avoid recomputing every time):
        self._cov_ori = [
            self.param.variance_orientation.value[0], 0.0, 0.0,
            0.0, self.param.variance_orientation.value[1], 0.0,
            0.0, 0.0, self.param.variance_orientation.value[2],
        ]
        self._cov_acc = [
            self.param.variance_acc.value[0], 0.0, 0.0,
            0.0, self.param.variance_acc.value[1], 0.0,
            0.0, 0.0, self.param.variance_acc.value[2],
        ]
        self._cov_gyr = [
            self.param.variance_angular_vel.value[0], 0.0, 0.0,
            0.0, self.param.variance_angular_vel.value[1], 0.0,
            0.0, 0.0, self.param.variance_angular_vel.value[2],
        ]
        self._cov_mag = [
            self.param.variance_mag.value[0], 0.0, 0.0,
            0.0, self.param.variance_mag.value[1], 0.0,
            0.0, 0.0, self.param.variance_mag.value[2],
        ]

        self._cov_unknown = [ 
            -1.0, 0.0, 0.0,
             0.0, -1.0, 0.0,
             0.0, 0.0, -1.0,
        ]

        # Set covariances once on reused messages
        self._imu_raw_msg.orientation_covariance = self._cov_ori
        self._imu_raw_msg.linear_acceleration_covariance = self._cov_acc
        self._imu_raw_msg.angular_velocity_covariance = self._cov_gyr

        self._imu_msg.orientation_covariance = self._cov_ori
        self._imu_msg.linear_acceleration_covariance = self._cov_acc
        self._imu_msg.angular_velocity_covariance = self._cov_gyr

        self._mag_msg.magnetic_field_covariance = self._cov_mag

        #
        # The sensor reading often contains invalid large jumps of the orientation w component.
        # To mitigate this, we implement a simple jump detection and filtering logic.
        # Publish only if the reading is valid according to jump detection logic
        #

        # Jump detection state variable:
        self._prev_q = None  # for sign continuity / jump checks

        # Temperature publish throttling:
        self._last_temp_publish_time = None  # Track last temperature publish time


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
        self.is_fusing_mode = self.param.operation_mode.value in (0x0B, 0x0C)
        # only NDOF_FMC_OFF or NDOF (with FMC) modes provide fused orientation data (but mag data reads zeroes)
        self.node.get_logger().info(f"Setting device_mode to {device_mode}  is_fusing_mode={self.is_fusing_mode}")

        if not self.is_fusing_mode:
            # In non-fusing modes, the orientation data is not valid, so set covariance to -1 to indicate unknown.
            self._imu_raw_msg.orientation_covariance = self._cov_unknown

        if not (self.con.transmit(registers.BNO055_OPR_MODE_ADDR, 1, bytes([device_mode]))):
            self.node.get_logger().warn('Unable to set IMU operation mode into operation mode.')

        self.node.get_logger().info('Bosch BNO055 IMU configuration complete.')


    def get_sensor_data(self):
        """Read IMU data from the sensor, parse and publish."""

        # avoid publishing/logging during shutdown
        if not rclpy.ok() or not self.node.context.ok():
            return

        # read from sensor: bytearray, 45 bytes starting from BNO055_ACCEL_DATA_X_LSB_ADDR
        buf = self.con.receive(registers.BNO055_ACCEL_DATA_X_LSB_ADDR, 45)

        if not buf or len(buf) != 45:
            self.node.get_logger().warn(f"Short read: got {len(buf) if buf else 0} bytes")
            return

        # if unsure about buffer type, copy the buffer to a bytes object
        #b = buf if isinstance(buf, (bytes, bytearray, memoryview)) else bytes(buf)
        b = buf

        # Unpack blocks (fewer Python calls)
        ax_raw, ay_raw, az_raw, mx_raw, my_raw, mz_raw, gx_raw, gy_raw, gz_raw = struct.unpack_from("<hhhhhhhhh", b, 0)
        lax_raw, lay_raw, laz_raw, grx_raw, gry_raw, grz_raw = struct.unpack_from("<hhhhhh", b, 32)
        temp_raw = struct.unpack_from("<b", b, 44)[0]  # signed

        # locals (avoid repeated attribute lookups)
        now_msg = self.node.get_clock().now().to_msg()
        frame_id = self.param.frame_id.value

        # dividers to convert raw sensor values to physical units, from driver parameters
        acc_div = self.param.acc_factor.value
        gyr_div = self.param.gyr_factor.value
        grav_div = self.param.grav_factor.value

        # only NDOF_FMC_OFF or NDOF (with FMC) modes provide fused orientation data (but mag data reads zeroes)
        if self.is_fusing_mode:

            qw_raw, qx_raw, qy_raw, qz_raw = struct.unpack_from("<hhhh", b, 24)

            # Normalize quaternion robustly; tolerate modes where it is invalid
            q = np.array([qx_raw, qy_raw, qz_raw, qw_raw], dtype=float)
            norm = float(np.linalg.norm(q))
            if norm < 1e-6 or abs(norm - 16384.0) > 2000.0:
                if self._prev_q is None:
                    self.node.get_logger().warn(f"Invalid quaternion norm: {norm} — skipped publishing imu message to avoid invalid data")
                    return  # no valid reading and no previous reading - skip publishing to avoid invalid data
                else:
                    self.node.get_logger().warn(f"Invalid quaternion norm: {norm} — publishing previous quaternion")
                    qn = self._prev_q  # hold last valid reading to mitigate jumps, if we have one
            else:
                qn = q / norm
                # sign continuity
                if self._prev_q is not None and float(np.dot(self._prev_q, qn)) < 0.0:
                    qn = -qn
                self._prev_q = qn

            # Header (single timestamp)
            self._imu_msg.header.stamp = now_msg
            self._imu_msg.header.frame_id = frame_id

            qx, qy, qz, qw = map(float, qn)   # explicit conversion
            self._imu_msg.orientation.x = qx
            self._imu_msg.orientation.y = qy
            self._imu_msg.orientation.z = qz
            self._imu_msg.orientation.w = qw

            # raw gyroscope data
            self._imu_msg.angular_velocity.x = gx_raw / gyr_div
            self._imu_msg.angular_velocity.y = gy_raw / gyr_div
            self._imu_msg.angular_velocity.z = gz_raw / gyr_div

            # “filtered/linear accel” block
            self._imu_msg.linear_acceleration.x = lax_raw / acc_div
            self._imu_msg.linear_acceleration.y = lay_raw / acc_div
            self._imu_msg.linear_acceleration.z = laz_raw / acc_div

            self.pub_imu.publish(self._imu_msg)

            # gravity block
            self._grav_msg.x = grx_raw / grav_div
            self._grav_msg.y = gry_raw / grav_div
            self._grav_msg.z = grz_raw / grav_div

            self.pub_grav.publish(self._grav_msg)  # Vector3 does not need header. Use Vector3Stamped if you need timestamp and frame_id

        else:
            # In other modes, we do not have fused orientation data, but have raw magnetometer data

            # Headers (same timestamp)
            self._imu_raw_msg.header.stamp = now_msg
            self._imu_raw_msg.header.frame_id = frame_id
            self._mag_msg.header.stamp = now_msg
            self._mag_msg.header.frame_id = frame_id

            # raw accelerometer data
            self._imu_raw_msg.linear_acceleration.x = ax_raw / acc_div
            self._imu_raw_msg.linear_acceleration.y = ay_raw / acc_div
            self._imu_raw_msg.linear_acceleration.z = az_raw / acc_div

            # raw gyroscope data
            self._imu_raw_msg.angular_velocity.x = gx_raw / gyr_div
            self._imu_raw_msg.angular_velocity.y = gy_raw / gyr_div
            self._imu_raw_msg.angular_velocity.z = gz_raw / gyr_div

            self.pub_imu_raw.publish(self._imu_raw_msg)

            # raw magnetometer data
            mag_div = self.param.mag_factor.value
            self._mag_msg.magnetic_field.x = mx_raw / mag_div  # 16 million LSB per Tesla, as per datasheet
            self._mag_msg.magnetic_field.y = my_raw / mag_div
            self._mag_msg.magnetic_field.z = mz_raw / mag_div

            self.pub_mag.publish(self._mag_msg)

        # Temperature - publish at most once per second
        current_time = self.node.get_clock().now()
        if self._last_temp_publish_time is None or \
           (current_time - self._last_temp_publish_time).nanoseconds >= 1_000_000_000:
            # Header
            self._temp_msg.header.stamp = now_msg
            self._temp_msg.header.frame_id = frame_id

            # temperature - one byte:
            self._temp_msg.temperature = float(temp_raw)  # a signed byte with unit 1 degree Celsius

            self.pub_temp.publish(self._temp_msg)
            self._last_temp_publish_time = current_time


    def get_calib_status(self):
        """
        Read calibration status for sys/gyro/acc/mag.

        Quality scale: 0 = bad, 3 = best
        """
        calib_status = self.con.receive(registers.BNO055_CALIB_STAT_ADDR, 1)
        c_sys = (calib_status[0] >> 6) & 0x03
        c_gyro = (calib_status[0] >> 4) & 0x03
        c_accel = (calib_status[0] >> 2) & 0x03
        c_mag = calib_status[0] & 0x03

        # Create dictionary (map) and convert it to JSON string:
        calib_status_dict = {'sys': c_sys, 'gyro': c_gyro, 'accel': c_accel, 'mag': c_mag}
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
