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
  - No parameters are supported as of yet. 
  - These will be implemented in the near future. 
  - Check back for updates soon!

**Publishes:**
- **bno055/imu** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
