# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
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
#  * Neither the name of Lucas Walter nor the names of its
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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="realsense_to_left_camera",
            arguments=[
                "-0.054934",
                "0",
                "-0.040939",
                "0",
                "-1.571",
                "0",
                "droid_optical_frame",
                "left_camera_optical_frame",
            ],
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="realsense_to_right_camera",
            arguments=[
                "0.031934",
                "0",
                "-0.040939",
                "0",
                "1.571",
                "0",
                "droid_optical_frame",
                "right_camera_optical_frame",
            ],
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_publisher",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "map",
                "odom",
            ],
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_map_droid_publisher",
            arguments=[
                "0",
                "0",
                "1.157",
                "-0.630",
                "0.630",
                "-0.322",
                "0.322",
                "odom",
                "map_droid"
            ],
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="droid_link_to_base_footprint_publisher",
            arguments=[
                "0",
                "0.938",
                "0.678",
                "0.630",
                "-0.630",
                "0.322",
                "0.322",
                "droid_optical_frame",
                "base_footprint",
            ],
        )
    )
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_footprint_to_ros2_camera_linkpublisher",
            arguments=[
                "0",
                "0",
                "1.157",
                "0",
                "0",
                "0",
                "1",
                "base_footprint",
                "ros2_camera_link",
            ],
        )
    )
    # ld.add_action(Node(
    #    package='usb_cam', executable='show_image.py', output='screen',
    # namespace=ns,
    # arguments=[image_manip_dir + "/data/mosaic.jpg"])
    # remappings=[('image_in', 'image_raw')]
    #    ))

    return ld
