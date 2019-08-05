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
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_default

import sys
import serial
from time import time
import binascii
import struct

#TODO: put these constant in a seperate file
# BOSCH BNO055 IMU Registers map and other information
# Page 0 registers
CHIP_ID = 0x00
PAGE_ID = 0x07
ACCEL_DATA = 0x08
MAG_DATA = 0x0e
GYRO_DATA = 0x14
FUSED_EULER = 0x1a
FUSED_QUAT = 0x20
LIA_DATA = 0x28
GRAVITY_DATA = 0x2e
TEMP_DATA = 0x34
CALIB_STAT = 0x35
SYS_STATUS = 0x39
SYS_ERR = 0x3a
UNIT_SEL = 0x3b
OPER_MODE = 0x3d
PWR_MODE = 0x3e
SYS_TRIGGER = 0x3f
TEMP_SOURCE = 0x440
AXIS_MAP_CONFIG = 0x41
AXIS_MAP_SIGN = 0x42

ACC_OFFSET = 0x55
MAG_OFFSET = 0x5b
GYR_OFFSET = 0x61
ACC_RADIUS = 0x68
MAG_RADIUS = 0x69

# Page 1 registers
ACC_CONFIG = 0x08
MAG_CONFIG = 0x09
GYR_CONFIG0 = 0x0a
GYR_CONFIG1 = 0x0b

#  Operation modes
OPER_MODE_CONFIG = 0x00
OPER_MODE_ACCONLY = 0x01
OPER_MODE_MAGONLY = 0x02
OPER_MODE_GYROONLY = 0x03
OPER_MODE_ACCMAG = 0x04
OPER_MODE_ACCGYRO = 0x05
OPER_MODE_MAGGYRO = 0x06
OPER_MODE_AMG = 0x07
OPER_MODE_IMU = 0x08
OPER_MODE_COMPASS = 0x09
OPER_MODE_M4G = 0x0a
OPER_MODE_NDOF_FMC_OFF = 0x0b
OPER_MODE_NDOF = 0x0C

#  Power modes
PWR_MODE_NORMAL = 0x00
PWR_MODE_LOW = 0x01
PWR_MODE_SUSPEND  = 0x02

# Communication constants
BNO055_ID = 0xa0
START_BYTE_WR = 0xaa
START_BYTE_RESP = 0xbb
READ = 0x01
WRITE = 0x00

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('bno055')
    pub_imu = node.create_publisher(Imu, 'imu', qos_profile_default)

    # Initialize ROS msgs
    imu_msg = Imu()

    # Initialize counters and constants
    seq = 0
    acc_fact = 1000.0
    mag_fact = 16.0
    gyr_fact = 900.0

#-----------------
    def open_serial(port, baudrate, timeout_):
        node.get_logger().info('Opening serial port: "%s"...' % port)
        try:
            usb_con = serial.Serial(port, baudrate, timeout=timeout_)
            return usb_con
        except serial.serialutil.SerialException:
            node.get_logger().info("IMU not found at port " + port + ". Check to make sure your device is connected.")
            sys.exit(0)
   
#------------------------
    def configure(usb_con):
        node.get_logger().info("Configuring device...")
        data = receive(usb_con, CHIP_ID, 1)
        #node.get_logger().info('device sent: "%s"' % data)
        if data == 0 or data[0] != BNO055_ID:
            node.get_logger().warn("Device ID is incorrect...shutting down.")
            sys.exit(0)

        # IMU connected, configure it
         # IMU Configuration
        if not(write(usb_con, OPER_MODE, 1, OPER_MODE_CONFIG)):
            node.get_logger().warn("Unable to set IMU into config mode.")

        if not(write(usb_con, PWR_MODE, 1, PWR_MODE_NORMAL)):
            node.get_logger().warn("Unable to set IMU normal power mode.")

        if not(write(usb_con, PAGE_ID, 1, 0x00)):
            node.get_logger().warn("Unable to set IMU register page 0.")

        if not(write(usb_con, SYS_TRIGGER, 1, 0x00)):
            node.get_logger().warn("Unable to start IMU.")

        if not(write(usb_con, UNIT_SEL, 1, 0x83)):
            node.get_logger().warn("Unable to set IMU units.")

        if not(write(usb_con, AXIS_MAP_CONFIG, 1, 0x24)):
            node.get_logger().warn("Unable to remap IMU axis.")

        if not(write(usb_con, AXIS_MAP_SIGN, 1, 0x06)):
            node.get_logger().warn("Unable to set IMU axis signs.")

        if not(write(usb_con, OPER_MODE, 1, OPER_MODE_NDOF)):
            node.get_logger().warn("Unable to set IMU operation mode into operation mode.")

        node.get_logger().info("Bosch BNO055 IMU configuration complete.")
        return 1 
#-----------------------
    # Read data from IMU
    def receive(usb_con, reg_addr, length):
        buf_out = bytearray()
        buf_out.append(START_BYTE_WR)
        buf_out.append(READ)
        buf_out.append(reg_addr)
        buf_out.append(length)

        try:
            usb_con.write(buf_out)
            buf_in = bytearray(usb_con.read(2 + length))
            #print("Reading, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
        except:
            return 0

        # Check if response is correct
        if (buf_in.__len__() != (2 + length)) or (buf_in[0] != START_BYTE_RESP):
            #node.get_logger().warn("Incorrect Bosh IMU device response.")
            return 0
        buf_in.pop(0)
        buf_in.pop(0)
        return buf_in
#-----------------------------
    def write(usb_con, reg_addr, length, data):
        buf_out = bytearray()
        buf_out.append(START_BYTE_WR)
        buf_out.append(WRITE)
        buf_out.append(reg_addr)
        buf_out.append(length)
        buf_out.append(data)

        try:
            usb_con.write(buf_out)
            buf_in = bytearray(usb_con.read(2))
            # print("Writing, wr: ", binascii.hexlify(buf_out), "  re: ", binascii.hexlify(buf_in))
        except:
            return False

        if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
            #rospy.logerr("Incorrect Bosh IMU device response.")
            return False
        return True
#--------------------------------------------

    # callback to read data from sensor
    def read_data():
        # read from sensor
        buf = receive(usb_con, ACCEL_DATA, 45)
        # Publish raw data
        # TODO: convert rcl Clock time to ros time?
        #imu_msg.header.stamp = node.get_clock().now()
        imu_msg.header.frame_id = 'bno055'
        # TODO: do headers need sequence counters now?
        #imu_msg.header.seq = seq
        if buf != 0:
            try:
                imu_msg.orientation_covariance[0] = -1
                imu_msg.linear_acceleration.x = float(struct.unpack('h', struct.pack('BB', buf[0], buf[1]))[0]) / acc_fact
                imu_msg.linear_acceleration.y = float(struct.unpack('h', struct.pack('BB', buf[2], buf[3]))[0]) / acc_fact
                imu_msg.linear_acceleration.z = float(struct.unpack('h', struct.pack('BB', buf[4], buf[5]))[0]) / acc_fact
                imu_msg.linear_acceleration_covariance[0] = -1
                imu_msg.angular_velocity.x = float(struct.unpack('h', struct.pack('BB', buf[12], buf[13]))[0]) / gyr_fact
                imu_msg.angular_velocity.y = float(struct.unpack('h', struct.pack('BB', buf[14], buf[15]))[0]) / gyr_fact
                imu_msg.angular_velocity.z = float(struct.unpack('h', struct.pack('BB', buf[16], buf[17]))[0]) / gyr_fact
                imu_msg.angular_velocity_covariance[0] = -1
                #node.get_logger().info('Publishing imu message')
                pub_imu.publish(imu_msg)
            except:
                # something went wrong...keep going to the next message
                node.get_logger().warn('oops')
    
    # try to connect
    usb_con = open_serial('/dev/ttyUSB0', 115200, 0.02)
    # configure imu
    if (configure(usb_con)):
        # successfully configured
        frequency = float(1 / 100)
        timer = node.create_timer(frequency, read_data)

    rclpy.spin(node)
    # clean shutdown
    shutdown()

def shutdown():
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
