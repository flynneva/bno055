from bno055.connectors.Connector import Connector
from bno055.params.NodeParameters import NodeParameters
from bno055 import registers

from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature, MagneticField
from rclpy.qos import QoSProfile

import struct
import sys
from time import time


class SensorService:
    """Provides an interface for accessing the sensor's features & data"""

    def __init__(self, node: Node, connector: Connector, param: NodeParameters):
        self.node = node
        self.connector = connector
        self.param = param

        # create topic publishers:
        # TODO make topics configurable or unique?
        self.pub_imu_raw = node.create_publisher(Imu, 'imu_raw', QoSProfile(depth=10))
        self.pub_imu = node.create_publisher(Imu, 'imu', QoSProfile(depth=10))
        self.pub_mag = node.create_publisher(MagneticField, 'mag', QoSProfile(depth=10))
        self.pub_temp = node.create_publisher(Temperature, 'temp', QoSProfile(depth=10))

    def configure(self):
        """
        Configures the IMU sensor hardware
        """

        self.node.get_logger().info("Configuring device...")
        data = self.connector.receive(registers.CHIP_ID, 1)
        #node.get_logger().info('device sent: "%s"' % data)
        if data == 0 or data[0] != registers.BNO055_ID:
            self.node.get_logger().warn("Device ID is incorrect...shutting down.")
            sys.exit(1)

        # IMU connected => apply IMU Configuration:
        if not(self.connector.transmit(registers.OPER_MODE, 1, registers.OPER_MODE_CONFIG)):
            self.node.get_logger().warn("Unable to set IMU into config mode.")

        if not(self.connector.transmit(registers.PWR_MODE, 1, registers.PWR_MODE_NORMAL)):
            self.node.get_logger().warn("Unable to set IMU normal power mode.")

        if not(self.connector.transmit(registers.PAGE_ID, 1, 0x00)):
            self.node.get_logger().warn("Unable to set IMU register page 0.")

        if not(self.connector.transmit(registers.SYS_TRIGGER, 1, 0x00)):
            self.node.get_logger().warn("Unable to start IMU.")

        if not(self.connector.transmit(registers.UNIT_SEL, 1, 0x83)):
            self.node.get_logger().warn("Unable to set IMU units.")

        if not(self.connector.transmit(registers.AXIS_MAP_CONFIG, 1, 0x24)):
            self.node.get_logger().warn("Unable to remap IMU axis.")

        if not(self.connector.transmit(registers.AXIS_MAP_SIGN, 1, 0x06)):
            self.node.get_logger().warn("Unable to set IMU axis signs.")

        if not(self.connector.transmit(registers.OPER_MODE, 1, registers.OPER_MODE_NDOF)):
            self.node.get_logger().warn("Unable to set IMU operation mode into operation mode.")

        self.node.get_logger().info("Bosch BNO055 IMU configuration complete.")

    def get_sensor_data(self):
        """
        Read IMU data from the sensor, parse and publish
        """

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

        # read from sensor
        buf = self.connector.receive(registers.ACCEL_DATA, 45)
        # Publish raw data
        # TODO: convert rcl Clock time to ros time?
        #imu_raw_msg.header.stamp = node.get_clock().now()
        imu_raw_msg.header.frame_id = self.param.frame_id.value
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
                self.pub_imu_raw.publish(imu_raw_msg)

                # TODO: make this an option to publish?
                # Publish filtered data
                #imu_msg.header.stamp = node.get_clock().now()
                imu_msg.header.frame_id = self.param.frame_id.value
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
                self.pub_imu.publish(imu_msg)

                # Publish magnetometer data
                #mag_msg.header.stamp = node.get_clock().now()
                mag_msg.header.frame_id = self.param.frame_id.value
                #mag_msg.header.seq = seq
                mag_msg.magnetic_field.x = float(struct.unpack('h', struct.pack('BB', buf[6], buf[7]))[0]) / mag_fact
                mag_msg.magnetic_field.y = float(struct.unpack('h', struct.pack('BB', buf[8], buf[9]))[0]) / mag_fact
                mag_msg.magnetic_field.z = float(struct.unpack('h', struct.pack('BB', buf[10], buf[11]))[0]) / mag_fact
                self.pub_mag.publish(mag_msg)

                # Publish temperature
                #temp_msg.header.stamp = node.get_clock().now()
                temp_msg.header.frame_id = self.param.frame_id.value
                #temp_msg.header.seq = seq
                temp_msg.temperature = float(buf[44])
                self.pub_temp.publish(temp_msg)
            except Exception as e:
                # something went wrong...keep going to the next message
                self.node.get_logger().warn('oops..something went wrong')
                self.node.get_logger().warn('Error: "%s"' % e)

    def get_calib_status(self):
        """
        Read calibration status for sys/gyro/acc/mag and display to screen (JK) (0 = bad, 3 = best)
        :return:
        """
        calib_status = self.connector.receive(registers.CALIB_STAT, 1)
        try:
            sys = (calib_status[0] >> 6) & 0x03
            gyro = (calib_status[0] >> 4) & 0x03;
            accel = (calib_status[0] >> 2) & 0x03;
            mag = calib_status[0] & 0x03;
            self.node.get_logger().info('Sys: %d, Gyro: %d, Accel: %d, Mag: %d' % (sys, gyro, accel, mag))
        except:
            self.node.get_logger().error('No calibration data received')

    # Read all calibration offsets and print to screen (JK)
    def get_calib_offsets(self):
        try:
            accel_offset_read = self.connector.receive(registers.ACC_OFFSET, 6)
            accel_offset_read_x = (accel_offset_read[1] << 8) | accel_offset_read[0]   # Combine MSB and LSB registers into one decimal
            accel_offset_read_y = (accel_offset_read[3] << 8) | accel_offset_read[2]   # Combine MSB and LSB registers into one decimal
            accel_offset_read_z = (accel_offset_read[5] << 8) | accel_offset_read[4]   # Combine MSB and LSB registers into one decimal

            mag_offset_read = self.connector.receive(registers.MAG_OFFSET, 6)
            mag_offset_read_x = (mag_offset_read[1] << 8) | mag_offset_read[0]   # Combine MSB and LSB registers into one decimal
            mag_offset_read_y = (mag_offset_read[3] << 8) | mag_offset_read[2]   # Combine MSB and LSB registers into one decimal
            mag_offset_read_z = (mag_offset_read[5] << 8) | mag_offset_read[4]   # Combine MSB and LSB registers into one decimal

            gyro_offset_read = self.connector.receive(registers.GYR_OFFSET, 6)
            gyro_offset_read_x = (gyro_offset_read[1] << 8) | gyro_offset_read[0]   # Combine MSB and LSB registers into one decimal
            gyro_offset_read_y = (gyro_offset_read[3] << 8) | gyro_offset_read[2]   # Combine MSB and LSB registers into one decimal
            gyro_offset_read_z = (gyro_offset_read[5] << 8) | gyro_offset_read[4]   # Combine MSB and LSB registers into one decimal

            self.node.get_logger().info('Accel offsets (x y z): %d %d %d, Mag offsets (x y z): %d %d %d, Gyro offsets (x y z): %d %d %d' % (accel_offset_read_x, accel_offset_read_y, accel_offset_read_z, mag_offset_read_x, mag_offset_read_y, mag_offset_read_z, gyro_offset_read_x, gyro_offset_read_y, gyro_offset_read_z))
        except:
            self.node.get_logger().error('Calibration data cannot be read')

    # --------------------------------------------
    # Write out calibration values (define as 16 bit signed hex)
    def set_calib_offsets (self, acc_offset, mag_offset, gyr_offset):
        # Must switch to config mode to write out
        if not(self.connector.transmit(registers.OPER_MODE, 1, registers.OPER_MODE_CONFIG)):
            self.node.get_logger().error('Unable to set IMU into config mode')
        time.sleep(0.025)

        # Seems to only work when writing 1 register at a time
        try:
            self.connector.transmit(registers.ACC_OFFSET, 1, acc_offset[0] & 0xFF)                # ACC X LSB
            self.connector.transmit(registers.ACC_OFFSET + 1, 1, (acc_offset[0] >> 8) & 0xFF)       # ACC X MSB
            self.connector.transmit(registers.ACC_OFFSET + 2, 1, acc_offset[1] & 0xFF)
            self.connector.transmit(registers.ACC_OFFSET + 3, 1, (acc_offset[1] >> 8) & 0xFF)
            self.connector.transmit(registers.ACC_OFFSET + 4, 1, acc_offset[2] & 0xFF)
            self.connector.transmit(registers.ACC_OFFSET + 5, 1, (acc_offset[2] >> 8) & 0xFF)

            self.connector.transmit(registers.MAG_OFFSET, 1, mag_offset[0] & 0xFF)
            self.connector.transmit(registers.MAG_OFFSET + 1, 1, (mag_offset[0] >> 8) & 0xFF)
            self.connector.transmit(registers.MAG_OFFSET + 2, 1, mag_offset[1] & 0xFF)
            self.connector.transmit(registers.MAG_OFFSET + 3, 1, (mag_offset[1] >> 8) & 0xFF)
            self.connector.transmit(registers.MAG_OFFSET + 4, 1, mag_offset[2] & 0xFF)
            self.connector.transmit(registers.MAG_OFFSET + 5, 1, (mag_offset[2] >> 8) & 0xFF)

            self.connector.transmit(registers.GYR_OFFSET, 1, gyr_offset[0] & 0xFF)
            self.connector.transmit(registers.GYR_OFFSET + 1, 1, (gyr_offset[0] >> 8) & 0xFF)
            self.connector.transmit(registers.GYR_OFFSET + 2, 1, gyr_offset[1] & 0xFF)
            self.connector.transmit(registers.GYR_OFFSET + 3, 1, (gyr_offset[1] >> 8) & 0xFF)
            self.connector.transmit(registers.GYR_OFFSET + 4, 1, gyr_offset[2] & 0xFF)
            self.connector.transmit(registers.GYR_OFFSET + 5, 1, (gyr_offset[2] >> 8) & 0xFF)
            return True
        except:
            return False
