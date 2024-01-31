#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelToStringNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_pub')
        self.publisher = self.create_publisher(
            String,
            '/chatter',
            10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # Convert Twist message to string
        msg_str = f"Linear: x={0.1}, y={0}, z={0}; Angular: x={0}, y={0}, z={0},"
        # Publish the string message
        self.publisher.publish(String(data=msg_str))
        self.get_logger().info('Publishing: "%s"' % msg_str)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToStringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
