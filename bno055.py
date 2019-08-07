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

# Default calibration values (taken from desk test approximation.) [x y z]
# Signed hex 16 bit representation

# +/- 2000 units (at max 2G)    (1 unit = 1 mg = 1 LSB = 0.01 m/s2)
ACC_OFFSET_DEFAULT = [0xFFEC, 0x00A5, 0xFFE8]
# +/- 6400 units                (1 unit = 1/16 uT)
MAG_OFFSET_DEFAULT = [0xFFB4, 0xFE9E, 0x027D]
# +/- 2000 units up to 32000 (dps range dependent)               (1 unit = 1/16 dps)
GYR_OFFSET_DEFAULT = [0x0002, 0xFFFF, 0xFFFF]

def main(args=None):
    rclpy.init()
    node = rclpy.create_node('bno055')
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

    baudrate = 115200

    try:
        # first, declare parameters that will be set
        node.declare_parameter('frame_id')
        node.declare_parameter('frequency')
        node.declare_parameter('port')
        node.declare_parameter('operation_mode')
        node.declare_parameter('acc_offset')
        node.declare_parameter('mag_offset')
        node.declare_parameter('gyr_offset')
        # then get the parameters
        node.get_logger().info('Parameters set to:')
        frame_id = node.get_parameter('frame_id')
        node.get_logger().info('    bno055/frame_id:    "%s"' % frame_id.value)
        port = node.get_parameter('port')
        node.get_logger().info('    bno055/port:        "%s"' % port.value)
        frequency = node.get_parameter('frequency')
        node.get_logger().info('    bno055/frequency:   "%s"' % frequency.value)
        operation_mode = node.get_parameter('operation_mode')
        node.get_logger().info('    bno055/operation_mode:    "%s"' % operation_mode.value)
        acc_offset = node.get_parameter('acc_offset')
        node.get_logger().info('    bno055/acc_offset:    "%s"' % acc_offset.value)
        mag_offset = node.get_parameter('mag_offset')
        node.get_logger().info('    bno055/mag_offset:    "%s"' % mag_offset.value)
        gyr_offset = node.get_parameter('gyr_offset')
        node.get_logger().info('    bno055/gyr_offset:    "%s"' % gyr_offset.value)
    except Exception as e:
         node.get_logger().warn('Could not get parameters...setting variables to default')
         node.get_logger().warn('Error: "%s"' % e)       
#        node.set_parameters( [
#            Parameter('frame_id', Parameter.Type.STRING, 'bno055'),
#            Parameter('port', Parameter.Type.STRING, '/dev/ttyUSB0'),
#            Parameter('frequency', Parameter.Type.DOUBLE, 100.0),
#            Parameter('baudrate', Parameter.Type.INTEGER, 115200) ])
#
#        node.get_logger().warn('Could not get parameters...setting variables to default')
#        node.get_logger().warn('Error: "%s"' % e)       
#        node.get_logger().info('Parameters set to:')
#        frame_id = node.get_parameter('frame_id')
#        node.get_logger().info('    bno055/frame_id:    "%s"' % frame_id.value)
#        port = node.get_parameter('port')
#        node.get_logger().info('    bno055/port:        "%s"' % port.value)
#        frequency = node.get_parameter('frequency')
#        node.get_logger().info('    bno055/frequency:   "%s"' % frequency.value)
#        baudrate = node.get_parameter('baudrate')
#        node.get_logger().info('    bno055/baudrate:    "%s"' % baudrate.value)
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
#TODO: might want to seperate this into its own seperate class for ROS2?
    # Read data from serial connection
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
            #node.get_logger().warn("Incorrect device response.")
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
    # Read calibration status for sys/gyro/acc/mag and display to screen (JK) (0 = bad, 3 = best)
    def get_calib_status (ser):
        calib_status = receive(ser, CALIB_STAT, 1)
        try:
            sys = (calib_status[0] >> 6) & 0x03
            gyro = (calib_status[0] >> 4) & 0x03;
            accel = (calib_status[0] >> 2) & 0x03;
            mag = calib_status[0] & 0x03; 
            rospy.loginfo('Sys: %d, Gyro: %d, Accel: %d, Mag: %d', sys, gyro, accel, mag)
        except:
            rospy.loginfo('No calibration data received')
#--------------------------------------------
# Read all calibration offsets and print to screen (JK)
    def get_calib_offsets (ser):
        try:
            accel_offset_read = receive(ser, ACC_OFFSET, 6)
            accel_offset_read_x = (accel_offset_read[1] << 8) | accel_offset_read[0]   # Combine MSB and LSB registers into one decimal
            accel_offset_read_y = (accel_offset_read[3] << 8) | accel_offset_read[2]   # Combine MSB and LSB registers into one decimal
            accel_offset_read_z = (accel_offset_read[5] << 8) | accel_offset_read[4]   # Combine MSB and LSB registers into one decimal

            mag_offset_read = receive(ser, MAG_OFFSET, 6)
            mag_offset_read_x = (mag_offset_read[1] << 8) | mag_offset_read[0]   # Combine MSB and LSB registers into one decimal
            mag_offset_read_y = (mag_offset_read[3] << 8) | mag_offset_read[2]   # Combine MSB and LSB registers into one decimal
            mag_offset_read_z = (mag_offset_read[5] << 8) | mag_offset_read[4]   # Combine MSB and LSB registers into one decimal

            gyro_offset_read = receive(ser, GYR_OFFSET, 6)
            gyro_offset_read_x = (gyro_offset_read[1] << 8) | gyro_offset_read[0]   # Combine MSB and LSB registers into one decimal
            gyro_offset_read_y = (gyro_offset_read[3] << 8) | gyro_offset_read[2]   # Combine MSB and LSB registers into one decimal
            gyro_offset_read_z = (gyro_offset_read[5] << 8) | gyro_offset_read[4]   # Combine MSB and LSB registers into one decimal

            rospy.loginfo('Accel offsets (x y z): %d %d %d, Mag offsets (x y z): %d %d %d, Gyro offsets (x y z): %d %d %d', accel_offset_read_x, accel_offset_read_y, accel_offset_read_z, mag_offset_read_x, mag_offset_read_y, mag_offset_read_z, gyro_offset_read_x, gyro_offset_read_y, gyro_offset_read_z)
        except:
            rospy.loginfo('Calibration data cant be read')
#--------------------------------------------
    # Write out calibration values (define as 16 bit signed hex)
    def set_calib_offsets (ser, acc_offset, mag_offset, gyr_offset):
        # Must switch to config mode to write out
        if not(write(ser, OPER_MODE, 1, OPER_MODE_CONFIG)):
           rospy.logerr("Unable to set IMU into config mode.")
        time.sleep(0.025)

        # Seems to only work when writing 1 register at a time
        try:
            write(ser, ACC_OFFSET, 1, acc_offset[0] & 0xFF)                # ACC X LSB
            write(ser, ACC_OFFSET+1, 1, (acc_offset[0] >> 8) & 0xFF)       # ACC X MSB
            write(ser, ACC_OFFSET+2, 1, acc_offset[1] & 0xFF)               
            write(ser, ACC_OFFSET+3, 1, (acc_offset[1] >> 8) & 0xFF)       
            write(ser, ACC_OFFSET+4, 1, acc_offset[2] & 0xFF)               
            write(ser, ACC_OFFSET+5, 1, (acc_offset[2] >> 8) & 0xFF)     

            write(ser, MAG_OFFSET, 1, mag_offset[0] & 0xFF)               
            write(ser, MAG_OFFSET+1, 1, (mag_offset[0] >> 8) & 0xFF)     
            write(ser, MAG_OFFSET+2, 1, mag_offset[1] & 0xFF)               
            write(ser, MAG_OFFSET+3, 1, (mag_offset[1] >> 8) & 0xFF)       
            write(ser, MAG_OFFSET+4, 1, mag_offset[2] & 0xFF)               
            write(ser, MAG_OFFSET+5, 1, (mag_offset[2] >> 8) & 0xFF)       

            write(ser, GYR_OFFSET, 1, gyr_offset[0] & 0xFF)                
            write(ser, GYR_OFFSET+1, 1, (gyr_offset[0] >> 8) & 0xFF)       
            write(ser, GYR_OFFSET+2, 1, gyr_offset[1] & 0xFF)               
            write(ser, GYR_OFFSET+3, 1, (gyr_offset[1] >> 8) & 0xFF)       
            write(ser, GYR_OFFSET+4, 1, gyr_offset[2] & 0xFF)               
            write(ser, GYR_OFFSET+5, 1, (gyr_offset[2] >> 8) & 0xFF)            

            return True
     
        except:
            return False
#--------------------------------------------
    # callback to read data from sensor
    def read_data():
        # read from sensor
        buf = receive(usb_con, ACCEL_DATA, 45)
        # Publish raw data
        # TODO: convert rcl Clock time to ros time?
        #imu_raw_msg.header.stamp = node.get_clock().now()
        imu_raw_msg.header.frame_id = frame_id.value
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
                imu_msg.header.frame_id = frame_id.value
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
                mag_msg.header.frame_id = frame_id.value
                #mag_msg.header.seq = seq
                mag_msg.magnetic_field.x = float(struct.unpack('h', struct.pack('BB', buf[6], buf[7]))[0]) / mag_fact
                mag_msg.magnetic_field.y = float(struct.unpack('h', struct.pack('BB', buf[8], buf[9]))[0]) / mag_fact
                mag_msg.magnetic_field.z = float(struct.unpack('h', struct.pack('BB', buf[10], buf[11]))[0]) / mag_fact
                pub_mag.publish(mag_msg)

                # Publish temperature
                #temp_msg.header.stamp = node.get_clock().now()
                temp_msg.header.frame_id = frame_id.value
                #temp_msg.header.seq = seq
                temp_msg.temperature = float(buf[44])
                pub_temp.publish(temp_msg)
            except Exception as e:
                # something went wrong...keep going to the next message
                node.get_logger().warn('oops..something went wrong')
                node.get_logger().warn('Error: "%s"' % e)
   
    
    # try to connect
    usb_con = open_serial(port.value, baudrate, 0.02)
    # configure imu
    if (configure(usb_con)):
        # successfully configured
        # TODO: this should be a parameter, given in Hz
        f = 1.0 / float(frequency.value)
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
