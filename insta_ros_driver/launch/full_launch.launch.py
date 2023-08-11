import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="insta_ros_driver",
                name="insta_fisheye",
                executable="insta_fisheye",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="insta_ros_driver",
                name="h264_subscriber1",
                executable="h264_subscriber",
                output="screen",
                remappings=[
                    ('/input/image','image_raw/h264_1'),
                    ('/output/image','/camera/image_raw1')
                ]
            ),
            launch_ros.actions.Node(
                package="insta_ros_driver",
                name="h264_subscriber2",
                executable="h264_subscriber",
                output="screen",
                remappings=[
                    ('/input/image','image_raw/h264_2'),
                    ('/output/image','/camera/image_raw2')
                ]
            )
        ]
    
    )