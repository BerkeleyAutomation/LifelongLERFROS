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
        self.color_sub = self.create_subscription(CompressedImage, '/camFront/image_raw/compressed', self.color_callback, 10)
        self.color_sub = self.create_subscription(CompressedImage, '/camRight/image_raw/compressed', self.color_callback, 10)
        self.color_sub = self.create_subscription(CompressedImage, '/camLeft/image_raw/compressed', self.color_callback, 10)
        self.color_sub = self.create_subscription(CompressedImage, '/camBack/image_raw/compressed', self.color_callback, 10)
        self.color_pub_front = self.create_publisher(Image, '/repub/camFront/image_raw', 10)
        self.color_pub_right = self.create_publisher(Image, '/repub/camRight/image_raw', 10)
        self.color_pub_left = self.create_publisher(Image, '/repub/camLeft/image_raw', 10)
        self.color_pub_back = self.create_publisher(Image, '/repub/camBack/image_raw', 10)
        self.cv_bridge = CvBridge()

    def color_callback(self, msg):
        try:
            color_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
            color_msg = self.cv_bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            color_msg.header = msg.header
            if msg.header.frame_id == 'camFront':
                self.color_pub_front.publish(color_msg)
            elif msg.header.frame_id == 'camRight':
                self.color_pub_right.publish(color_msg)
            elif msg.header.frame_id == 'camLeft':
                self.color_pub_left.publish(color_msg)
            elif msg.header.frame_id == 'camBack':
                self.color_pub_back.publish(color_msg)
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