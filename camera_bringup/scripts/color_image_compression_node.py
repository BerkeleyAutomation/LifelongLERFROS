#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge

class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        self.subscriber = self.create_subscription(
            Image,
            '/ros2_camera/color/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            CompressedImage,
            '/imageo_compressedo',
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        compressed_image = self.bridge.cv2_to_compressed_imgmsg(cv_image)
        compressed_image.header = msg.header
        # Publish compressed image
        self.publisher.publish(compressed_image)

def main(args=None):
    rclpy.init(args=args)
    image_compressor = ImageCompressor()
    rclpy.spin(image_compressor)
    image_compressor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
