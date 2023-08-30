import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                #node_name="base_laser_tf_publisher",
                arguments=[
                    "0.", 
                    "0",
                    "0.78105", 
                    "-3.14159",
                    "0", 
                    "1.5707963",
                    "base_link_realsense",
                    "camera",])
        ]
    )
