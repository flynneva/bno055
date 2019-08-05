## bno055
A ROS2 driver for the sensor IMU Bosch BNO055. It was implemented only the UART communication interface
(correct sensor mode should be selected...see diagram below).

This repo was based off of [Michael Drwiega's work on the Bosch IMU Driver for ROS 1](https://github.com/mdrwiega/bosch_imu_driver)

**Wiring Guide:**
- Within BNO055: 3vo=PS1 to select UART.

| BNO055 | CP2104 Friend    |
| ------ | ---------------- |
| Vin    |   5V             |
| GND    |   GND            |
| SDA    |   RXD            |
| SCL    |   TXD            |

**NOTE: on the CP2104 the pins above refer to the FTDI pins at the opposite end from the USB connector

**Parameters:**
- **port** (default: '/dev/ttyUSB0') - path to USB port where device was connected.
- **frame_id** (default: 'imu_link') - the frame in which sensor data will be published. 
- **frequency** (default: 100) - the frequency of reading from device and publishing data in Hz.
- **operation_mode** (default: OPER_MODE_NDOF) - the operation mode of sensor BNO055. Other modes could be found in sensor [datasheet](https://www.bosch-sensortec.com/bst/products/all_products/bno055).

Publishes:
- **/imu/data** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
- **/imu/raw** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
- **/imu/mag** [(sensor_msgs/MagneticField)](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html)
- **/imu/temp** [(sensor_msgs/Temperature)](http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html)
