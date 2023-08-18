from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description_pkg_share = FindPackageShare(package="navigation_bringup").find("navigation_bringup")
    rviz_path = os.path.join(description_pkg_share,'config','navigation.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [rviz_path]]
        )
    ])