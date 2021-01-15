# The bno055 ROS2 package

## Description
A ROS2 driver for the sensor IMU Bosch BNO055. It was implemented only the UART communication interface
(correct sensor mode should be selected...see diagram below).

This repo was based off of [Michael Drwiega's work on the Bosch IMU Driver for ROS 1](https://github.com/mdrwiega/bosch_imu_driver)

## Wiring Guide

### Selecting Connection Type

The default mode is I2C. Connect Pin PS1 with 3.3V to select UART mode. 

### CP2104 USB-to-UART Bridge

When using a CP2104 USB-to-UART Bridge:

| BNO055 | CP2104 Friend    |
| ------ | ---------------- |
| Vin    |   5V             |
| GND    |   GND            |
| SDA    |   RXD            |
| SCL    |   TXD            |

**NOTE: on the CP2104 the pins above refer to the FTDI pins at the opposite end from the USB connector

## ROS Node Parameters

In order to configuration, please adjust the [node parameter file](bno055/params/bno055_params.yaml) and pass it
as parameter when starting the node:

    ros2 run bno055 bno055 --ros-args --params-file ./src/bno055/bno055/params/bno055_params.yaml

### UART Connection

- **connection_type=uart**: Defines UART as sensor connection type; default='uart'
- **uart_port**: The UART port to use; default='/dev/ttyUSB0'
- **uart_baudrate**: The baud rate to use; default=115200
- **uart_timeout**: The timeout for UART transmissions in seconds to use; default=0.1
  
### Sensor Configuration

- **frame_id**: coordinate frame id of sensor default='bno055'
- **baudrate**: baudrate of sensor default=115200
- **data_query_frequency**: frequency to read data from sensor default=100 Hz

## ROS Topics

ROS topics published by this ROS2 Node: 

  - **bno055/imu** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
  - **bno055/imu_raw** [(sensor_msgs/Imu)](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
  - **bno055/temp** [(sensor_msgs/Temperature)](http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html)
  - **bno055/mag** [(sensor_msgs/MagneticField)](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html)
  
## Development Workspace Setup

### On a Remote Device
Setup of a ROS2 workspace & IDE for a remote device (for example Raspberry Pi):

#### Clone & Build

Create a ROS2 [workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/) on your remote device - for instance `~/ros2_ws`

Make sure you sourced your ROS2 installation (underlay).

Then clone the project into your workspace's src directory:

    cd ~/ros2_ws/src
    git clone https://github.com/flynneva/bno055.git
    
Perform a build of your workspace
    
    cd ~/ros2_ws
    colcon build

#### Integrate in your IDE

In order to work with the sources in your remote workspace and to integrate them in your IDE, use `sshfs`:

    sudo apt-get install sshfs
    sudo modprobe fuse

Create a IDE project directory and mount the remote ROS2 workspace:

    mkdir -p ~/projects/bno055/ros2_ws
    sshfs ubuntu@192.168.2.153:~/ros2_ws ~/projects/bno055/ros2_ws
    
Create a new project in your IDE from existing sources in `~/projects/bno055/ros2_ws`. 
You can now manipulate the remote ROS2 workspace using your local IDE (including git operations). 

### Running the ROS2 node
Run the `bno055` ROS2 node with default parameters:

    # source your local workspace (overlay) in addition to the ROS2 sourcing (underlay):
    source ~/ros2_ws/install/setup.sh
    # run the node:
    ros2 run bno055 bno055
    
Run with customized parameter file:

    ros2 run bno055 bno055 --ros-args --params-file ./src/bno055/bno055/params/bno055_params.yaml
    
### Performing flake8 Linting

To perform code linting with [flake8](https://gitlab.com/pycqa/flake8) just perform:

    cd ~/ros2_ws/src/bno055
    ament_flake8

See [www.flake8rules.com](https://www.flake8rules.com/) for more detailed information about flake8 rules


