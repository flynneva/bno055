# The bno055 ROS2 package

## Description
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
Run the `bno055` ROS2 node:

    # source your local workspace (overlay) in addition to the ROS2 sourcing (underlay):
    source ~/ros2_ws/install/setup.sh
    # run the node:
    ros2 run bno055 bno055
    
    



