## The bno055 ROS2 package
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
  - **port**: /path/to/device/port default='/dev/ttyUSB0'
  - **frame_id**: coordinate frame id of sensor default='bno055'
  - **baudrate**: baudrate of sensor default=115200
  - **frequency**: frequency to read data from sensor default=100 Hz

**Publishes:**
  - **bno055/imu** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
  - **bno055/imu_raw** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
  - **bno055/temp** [(sensor_msgs/Temperature)](http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html)
  - **bno055/mag** [(sensor_msgs/MagneticField)](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html)
