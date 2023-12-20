#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class RealsenseCompression(Node):
    def __init__(self):
        super().__init__('realsense_compressor')
        self.color_sub = self.create_subscription(Image, '/ros2_camera/color/image_raw', self.color_callback, 10)
        self.color_pub = self.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', 10)
        self.cv_bridge = CvBridge()

    def color_callback(self, msg):
        try:
            color_image = self.cv_bridge.imgmsg_to_cv2(msg)
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            color_msg = self.cv_bridge.cv2_to_compressed_imgmsg(color_image)
            color_msg.header = msg.header
            self.color_pub.publish(color_msg)
        except Exception as e:
            self.get_logger().error('Error republishing color image: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseCompression()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()