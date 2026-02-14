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
 
# colcon build; source install/setup.bash; ros2 launch bno055 bno055_raw.launch.py

# see https://github.com/slgrobotics/articubot_one/blob/main/robots/turtle/launch/turtle.sensors.launch.py
#     https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO055%20IMU.md

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    namespace = ""

    ld = LaunchDescription()
        
    bno055_driver_raw_node = Node(
        package='bno055',
        namespace=namespace,
        executable='bno055',
        name='bno055',
        output='screen',
        respawn=True,
        respawn_delay=4,
        parameters=[{
            # see https://github.com/slgrobotics/bno055
            #     https://github.com/slgrobotics/robots_bringup/blob/main/Docs/Sensors/BNO055%20IMU.md
            'ros_topic_prefix': 'imu/',
            'connection_type': 'i2c',
            'i2c_bus': 1,
            'i2c_addr': [0x29,0x28],   # Adafruit - 0x28, GY Clone - 0x29 (with both jumpers closed)
            'data_query_frequency': 30,
            'calib_status_frequency': 1.0,
            'frame_id': 'imu_link',
            # Fast Magnetometer Calibration mode (FMC) provides faster magnetometer calibration
            #  at the cost of slightly higher noise. NDOF_FMC_OFF is the default mode with slower calibration but lower noise.
            #  ACCGYRO and MAGGYRO modes provide raw accelerometer and gyroscope data without sensor fusion,
            #  which can be useful for certain applications but may require additional processing to obtain orientation data.
            'operation_mode': 0x07, # 0x0C = NDOF (with FMC), 0x0B - NDOF_FMC_OFF, 0x07 - ACC+GYRO+MAG (AMG)
            'placement_axis_remap': 'P1', # P1 - default, ENU. See Bosch BNO055 datasheet section "Axis Remap"
            'acc_factor': 100.0,
            'mag_factor': 16000000.0,
            'gyr_factor': 900.0,
            'grav_factor': 100.0,
            'set_offsets': False, # set to true to use offsets below
            'offset_acc': [0xFFEC, 0x00A5, 0xFFE8],
            'offset_gyr': [0x0002, 0xFFFF, 0xFFFF],
            'offset_mag': [0xFFB4, 0xFE9E, 0x027D],
            'radius_mag': 800,   # means 800 microtesla per LSB
            'radius_acc': 1000,  # means 1G = 1000 units LSB
            # Sensor standard deviation [x,y,z]
            # Used to calculate covariance matrices
            # driver defaults are used if parameters below are not provided - bno055/src/bno055/bno055/registers.py:255
            # see https://chatgpt.com/s/t_691b60f38e1c8191a0a309cbcf99e478
            'variance_acc': [0.017, 0.017, 0.017],  # [m/s^2]      defaults: [0.017, 0.017, 0.017]
            'variance_angular_vel': [0.04, 0.04, 0.04],  # [rad/s] defaults: [0.04, 0.04, 0.04]
            'variance_orientation': [0.0159, 0.0159, 0.0159],  # [rad] - (roll, pitch, yaw)  defaults: [0.0159, 0.0159, 0.0159]
            'variance_mag': [-1.0, 0.0, 0.0],  # [Tesla]           defaults: [-1.0, 0.0, 0.0] - "unknown" covariance, see REP 117
        }],
        remappings=[("imu/imu", "imu/data"), ("imu/imu_raw", "imu/data_raw")]
    )

    # Madgwick filter node to compute orientation quaternion from raw IMU data
    # publishes to "imu/data" topic
    # https://github.com/CCNYRoboticsLab/imu_tools
    # sudo apt install ros-${ROS_DISTRO}-imu-tools
    madgwick_node =Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            "stateless": False,
            "use_mag": True,
            "publish_tf": True,
            "reverse_tf": False,
            "fixed_frame": "odom",
            "constant_dt": 0.0,
            "publish_debug_topics": False,
            "world_frame": "enu",
            "gain": 0.1,
            "zeta": 0.0,
            "mag_bias_x": 0.0,
            "mag_bias_y": 0.0,
            "mag_bias_z": 0.0,
            "orientation_stddev": 0.0
        }],
        #remappings=[("imu/mag", "imu/mag"), ("imu/data_raw", "imu/data_raw"), ("imu/data", "imu/data")],
    )

    # for experiments: RViz starts with "map" as Global Fixed Frame, provide a TF to see axes etc.
    tf = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments=[
            '--x', '0.0',     # X translation in meters
            '--y', '0.0',     # Y translation in meters
            '--z', '0.1',     # Z translation in meters
            '--roll', '0.0',  # Roll in radians
            '--pitch', '0.0', # Pitch in radians
            '--yaw', '0.0',   # Yaw in radians (e.g., 90 degrees)
            '--frame-id', 'map', # Parent frame ID
            '--child-frame-id', 'odom' # Child frame ID
        ]
    )
    ld.add_action(bno055_driver_raw_node)
    ld.add_action(madgwick_node)
    ld.add_action(tf)
    return ld

