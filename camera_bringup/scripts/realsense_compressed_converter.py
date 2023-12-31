#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageRepublisherNode(Node):
    def __init__(self):
        super().__init__('image_republisher')
        self.color_sub = self.create_subscription(CompressedImage, '/imageo_compressedo', self.color_callback, 10)
        self.color_pub = self.create_publisher(Image, '/repub_image_raw', 10)
        self.cv_bridge = CvBridge()

    def color_callback(self, msg):
        try:
            color_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
            color_msg = self.cv_bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            color_msg.header = msg.header
            self.color_pub.publish(color_msg)
        except Exception as e:
            self.get_logger().error('Error republishing color image: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    node = ImageRepublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()