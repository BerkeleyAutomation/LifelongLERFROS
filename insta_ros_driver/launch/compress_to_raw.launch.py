from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            name='compress_to_raw',
            arguments=['compressed', 'raw'],
            remappings=[
                ('/in/compressed', '/image_raw/compressed'),
                ('out', '/new_raw')
            ]
        )
    ])
