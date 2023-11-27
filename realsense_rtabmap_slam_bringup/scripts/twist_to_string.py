#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelToStringNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_string_converter')
        self.subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(
            String,
            'chatter',
            10)

    def cmd_vel_callback(self, msg):
        # Convert Twist message to string
        msg_str = f"Linear: x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z}; Angular: x={msg.angular.x}, y={msg.angular.y}, z={msg.angular.z},"
        # Publish the string message
        self.publisher.publish(String(data=msg_str))

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToStringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
