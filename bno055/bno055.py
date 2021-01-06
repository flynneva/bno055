#!/usr/bin/env python3
#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Michal Drwiega
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#####################################################################

import rclpy
from sensor_msgs.msg import Imu, Temperature, MagneticField
from rclpy.qos import QoSProfile
#from rclpy.parameter import Parameter

import sys
from time import time
import binascii
import struct

from bno055 import bno055_registers
from bno055.connectors.UARTConnector import UARTConnector
from bno055.connectors.I2CConnector import I2CConnector
from bno055.params.NodeParameters import Parameters

def main(args=None):
    rclpy.init()
    node = rclpy.create_node('bno055')

    # create topic publishers:
    pub_imu_raw = node.create_publisher(Imu, 'imu_raw', QoSProfile(depth=10))
    pub_imu = node.create_publisher(Imu, 'imu', QoSProfile(depth=10))
    pub_mag = node.create_publisher(MagneticField, 'mag', QoSProfile(depth=10))
    pub_temp = node.create_publisher(Temperature, 'temp', QoSProfile(depth=10))

    # Initialize ROS msgs
    imu_raw_msg = Imu()
    imu_msg = Imu()
    mag_msg = MagneticField()
    temp_msg = Temperature()

    # Initialize counters and constants
    seq = 0
    acc_fact = 1000.0
    mag_fact = 16.0
    gyr_fact = 900.0

    # Initialize Parameters:
    param = Parameters(node)

    # ------------------------
    def configure():
        node.get_logger().info("Configuring device...")
        data = connector.receive(bno055_registers.CHIP_ID, 1)
        #node.get_logger().info('device sent: "%s"' % data)
        if data == 0 or data[0] != bno055_registers.BNO055_ID:
            node.get_logger().warn("Device ID is incorrect...shutting down.")
            sys.exit(1)

        # IMU connected, configure it
        # IMU Configuration
        if not(connector.transmit(bno055_registers.OPER_MODE, 1, bno055_registers.OPER_MODE_CONFIG)):
            node.get_logger().warn("Unable to set IMU into config mode.")

        if not(connector.transmit(bno055_registers.PWR_MODE, 1, bno055_registers.PWR_MODE_NORMAL)):
            node.get_logger().warn("Unable to set IMU normal power mode.")

        if not(connector.transmit(bno055_registers.PAGE_ID, 1, 0x00)):
            node.get_logger().warn("Unable to set IMU register page 0.")

        if not(connector.transmit(bno055_registers.SYS_TRIGGER, 1, 0x00)):
            node.get_logger().warn("Unable to start IMU.")

        if not(connector.transmit(bno055_registers.UNIT_SEL, 1, 0x83)):
            node.get_logger().warn("Unable to set IMU units.")

        if not(connector.transmit(bno055_registers.AXIS_MAP_CONFIG, 1, 0x24)):
            node.get_logger().warn("Unable to remap IMU axis.")

        if not(connector.transmit(bno055_registers.AXIS_MAP_SIGN, 1, 0x06)):
            node.get_logger().warn("Unable to set IMU axis signs.")

        if not(connector.transmit(bno055_registers.OPER_MODE, 1, bno055_registers.OPER_MODE_NDOF)):
            node.get_logger().warn("Unable to set IMU operation mode into operation mode.")

        node.get_logger().info("Bosch BNO055 IMU configuration complete.")
        return 1


    # --------------------------------------------
    # Read calibration status for sys/gyro/acc/mag and display to screen (JK) (0 = bad, 3 = best)
    def get_calib_status ():
        calib_status = connector.receive(bno055_registers.CALIB_STAT, 1)
        try:
            sys = (calib_status[0] >> 6) & 0x03
            gyro = (calib_status[0] >> 4) & 0x03;
            accel = (calib_status[0] >> 2) & 0x03;
            mag = calib_status[0] & 0x03;
            rospy.loginfo('Sys: %d, Gyro: %d, Accel: %d, Mag: %d', sys, gyro, accel, mag)
        except:
            rospy.loginfo('No calibration data received')

    # --------------------------------------------
    # Read all calibration offsets and print to screen (JK)
    def get_calib_offsets():
        try:
            accel_offset_read = connector.receive(bno055_registers.ACC_OFFSET, 6)
            accel_offset_read_x = (accel_offset_read[1] << 8) | accel_offset_read[0]   # Combine MSB and LSB registers into one decimal
            accel_offset_read_y = (accel_offset_read[3] << 8) | accel_offset_read[2]   # Combine MSB and LSB registers into one decimal
            accel_offset_read_z = (accel_offset_read[5] << 8) | accel_offset_read[4]   # Combine MSB and LSB registers into one decimal

            mag_offset_read = connector.receive(bno055_registers.MAG_OFFSET, 6)
            mag_offset_read_x = (mag_offset_read[1] << 8) | mag_offset_read[0]   # Combine MSB and LSB registers into one decimal
            mag_offset_read_y = (mag_offset_read[3] << 8) | mag_offset_read[2]   # Combine MSB and LSB registers into one decimal
            mag_offset_read_z = (mag_offset_read[5] << 8) | mag_offset_read[4]   # Combine MSB and LSB registers into one decimal

            gyro_offset_read = connector.receive(bno055_registers.GYR_OFFSET, 6)
            gyro_offset_read_x = (gyro_offset_read[1] << 8) | gyro_offset_read[0]   # Combine MSB and LSB registers into one decimal
            gyro_offset_read_y = (gyro_offset_read[3] << 8) | gyro_offset_read[2]   # Combine MSB and LSB registers into one decimal
            gyro_offset_read_z = (gyro_offset_read[5] << 8) | gyro_offset_read[4]   # Combine MSB and LSB registers into one decimal

            rospy.loginfo('Accel offsets (x y z): %d %d %d, Mag offsets (x y z): %d %d %d, Gyro offsets (x y z): %d %d %d', accel_offset_read_x, accel_offset_read_y, accel_offset_read_z, mag_offset_read_x, mag_offset_read_y, mag_offset_read_z, gyro_offset_read_x, gyro_offset_read_y, gyro_offset_read_z)
        except:
            rospy.loginfo('Calibration data cant be read')

    # --------------------------------------------
    # Write out calibration values (define as 16 bit signed hex)
    def set_calib_offsets (acc_offset, mag_offset, gyr_offset):
        # Must switch to config mode to write out
        if not(connector.transmit(bno055_registers.OPER_MODE, 1, bno055_registers.OPER_MODE_CONFIG)):
            rospy.logerr("Unable to set IMU into config mode.")
        time.sleep(0.025)

        # Seems to only work when writing 1 register at a time
        try:
            connector.transmit(bno055_registers.ACC_OFFSET, 1, acc_offset[0] & 0xFF)                # ACC X LSB
            connector.transmit(bno055_registers.ACC_OFFSET + 1, 1, (acc_offset[0] >> 8) & 0xFF)       # ACC X MSB
            connector.transmit(bno055_registers.ACC_OFFSET + 2, 1, acc_offset[1] & 0xFF)
            connector.transmit(bno055_registers.ACC_OFFSET + 3, 1, (acc_offset[1] >> 8) & 0xFF)
            connector.transmit(bno055_registers.ACC_OFFSET + 4, 1, acc_offset[2] & 0xFF)
            connector.transmit(bno055_registers.ACC_OFFSET + 5, 1, (acc_offset[2] >> 8) & 0xFF)

            connector.transmit(bno055_registers.MAG_OFFSET, 1, mag_offset[0] & 0xFF)
            connector.transmit(bno055_registers.MAG_OFFSET + 1, 1, (mag_offset[0] >> 8) & 0xFF)
            connector.transmit(bno055_registers.MAG_OFFSET + 2, 1, mag_offset[1] & 0xFF)
            connector.transmit(bno055_registers.MAG_OFFSET + 3, 1, (mag_offset[1] >> 8) & 0xFF)
            connector.transmit(bno055_registers.MAG_OFFSET + 4, 1, mag_offset[2] & 0xFF)
            connector.transmit(bno055_registers.MAG_OFFSET + 5, 1, (mag_offset[2] >> 8) & 0xFF)

            connector.transmit(bno055_registers.GYR_OFFSET, 1, gyr_offset[0] & 0xFF)
            connector.transmit(bno055_registers.GYR_OFFSET + 1, 1, (gyr_offset[0] >> 8) & 0xFF)
            connector.transmit(bno055_registers.GYR_OFFSET + 2, 1, gyr_offset[1] & 0xFF)
            connector.transmit(bno055_registers.GYR_OFFSET + 3, 1, (gyr_offset[1] >> 8) & 0xFF)
            connector.transmit(bno055_registers.GYR_OFFSET + 4, 1, gyr_offset[2] & 0xFF)
            connector.transmit(bno055_registers.GYR_OFFSET + 5, 1, (gyr_offset[2] >> 8) & 0xFF)

            return True

        except:
            return False

    # --------------------------------------------
    # callback to read data from sensor
    def read_data():
        # read from sensor
        buf = connector.receive(bno055_registers.ACCEL_DATA, 45)
        # Publish raw data
        # TODO: convert rcl Clock time to ros time?
        #imu_raw_msg.header.stamp = node.get_clock().now()
        imu_raw_msg.header.frame_id = param.frame_id.value
        # TODO: do headers need sequence counters now?
        #imu_raw_msg.header.seq = seq
        if buf != 0:
            try:
                # TODO: make this an option to publish?
                imu_raw_msg.orientation_covariance[0] = -1
                imu_raw_msg.linear_acceleration.x = float(struct.unpack('h', struct.pack('BB', buf[0], buf[1]))[0]) / acc_fact
                imu_raw_msg.linear_acceleration.y = float(struct.unpack('h', struct.pack('BB', buf[2], buf[3]))[0]) / acc_fact
                imu_raw_msg.linear_acceleration.z = float(struct.unpack('h', struct.pack('BB', buf[4], buf[5]))[0]) / acc_fact
                imu_raw_msg.linear_acceleration_covariance[0] = -1
                imu_raw_msg.angular_velocity.x = float(struct.unpack('h', struct.pack('BB', buf[12], buf[13]))[0]) / gyr_fact
                imu_raw_msg.angular_velocity.y = float(struct.unpack('h', struct.pack('BB', buf[14], buf[15]))[0]) / gyr_fact
                imu_raw_msg.angular_velocity.z = float(struct.unpack('h', struct.pack('BB', buf[16], buf[17]))[0]) / gyr_fact
                imu_raw_msg.angular_velocity_covariance[0] = -1
                #node.get_logger().info('Publishing imu message')
                pub_imu_raw.publish(imu_raw_msg)

                # TODO: make this an option to publish?
                # Publish filtered data
                #imu_msg.header.stamp = node.get_clock().now()
                imu_msg.header.frame_id = param.frame_id.value
                #imu_msg.header.seq = seq
                imu_msg.orientation.w = float(struct.unpack('h', struct.pack('BB', buf[24], buf[25]))[0])
                imu_msg.orientation.x = float(struct.unpack('h', struct.pack('BB', buf[26], buf[27]))[0])
                imu_msg.orientation.y = float(struct.unpack('h', struct.pack('BB', buf[28], buf[29]))[0])
                imu_msg.orientation.z = float(struct.unpack('h', struct.pack('BB', buf[30], buf[31]))[0])
                imu_msg.linear_acceleration.x = float(struct.unpack('h', struct.pack('BB', buf[32], buf[33]))[0]) / acc_fact
                imu_msg.linear_acceleration.y = float(struct.unpack('h', struct.pack('BB', buf[34], buf[35]))[0]) / acc_fact
                imu_msg.linear_acceleration.z = float(struct.unpack('h', struct.pack('BB', buf[36], buf[37]))[0]) / acc_fact
                imu_msg.linear_acceleration_covariance[0] = -1
                imu_msg.angular_velocity.x = float(struct.unpack('h', struct.pack('BB', buf[12], buf[13]))[0]) / gyr_fact
                imu_msg.angular_velocity.y = float(struct.unpack('h', struct.pack('BB', buf[14], buf[15]))[0]) / gyr_fact
                imu_msg.angular_velocity.z = float(struct.unpack('h', struct.pack('BB', buf[16], buf[17]))[0]) / gyr_fact
                imu_msg.angular_velocity_covariance[0] = -1
                pub_imu.publish(imu_msg)

                # Publish magnetometer data
                #mag_msg.header.stamp = node.get_clock().now()
                mag_msg.header.frame_id = param.frame_id.value
                #mag_msg.header.seq = seq
                mag_msg.magnetic_field.x = float(struct.unpack('h', struct.pack('BB', buf[6], buf[7]))[0]) / mag_fact
                mag_msg.magnetic_field.y = float(struct.unpack('h', struct.pack('BB', buf[8], buf[9]))[0]) / mag_fact
                mag_msg.magnetic_field.z = float(struct.unpack('h', struct.pack('BB', buf[10], buf[11]))[0]) / mag_fact
                pub_mag.publish(mag_msg)

                # Publish temperature
                #temp_msg.header.stamp = node.get_clock().now()
                temp_msg.header.frame_id = param.frame_id.value
                #temp_msg.header.seq = seq
                temp_msg.temperature = float(buf[44])
                pub_temp.publish(temp_msg)
            except Exception as e:
                # something went wrong...keep going to the next message
                node.get_logger().warn('oops..something went wrong')
                node.get_logger().warn('Error: "%s"' % e)

    # Get connector according to configured sensor connection type:
    if param.connection_type.value == UARTConnector.CONNECTIONTYPE_UART:
        connector = UARTConnector(node, param.baudrate.value, param.port.value, 0.02)
    elif param.connection_type.value == I2CConnector.CONNECTIONTYPE_I2C:
        # TODO implement IC2 integration
        node.get_logger().error('I2C not yet supported')
        sys.exit(1)
    else:
        node.get_logger().error('Unsupported connection type: ' + str(param.connection_type.value))
        sys.exit(1)

    # Connect to BNO055 device:
    connector.connect()

    # configure imu
    configure()

    # start regular sensor transmissions:
    f = 1.0 / float(param.frequency.value)
    timer = node.create_timer(f, read_data)

    rclpy.spin(node)
    # clean shutdown
    shutdown()

def shutdown():
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
