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
 
# colcon build; source install/setup.bash; ros2 launch bno055 bno055_I2C.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('bno055'),
        'config',
        'bno055_params_i2c.yaml'
        )
        
    node = Node(
        package = 'bno055',
        executable = 'bno055',
        namespace = '',
        parameters = [config],
        remappings=[("bno055/imu", "imu/data"),
                    ("bno055/imu_raw", "imu/data_raw"),  
                    ("bno055/mag","imu/mag"), 
                    ("bno055/temp", "imu/temp"), 
                    ("bno055/grav", "imu/grav"), 
                    ("bno055/calib_status", "imu/calib_status") 
        ]
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
            '--child-frame-id', 'imu_link' # Child frame ID
        ]
    )

    ld.add_action(node)
    ld.add_action(tf)
    return ld