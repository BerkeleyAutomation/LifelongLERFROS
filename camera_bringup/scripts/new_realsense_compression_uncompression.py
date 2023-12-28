#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ImageCompressorNode(Node):
    def __init__(self):
        super().__init__('image_compressor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(CompressedImage, '/camera/color/image_raw/compressed', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        compressed_image = self.bridge.cv2_to_compressed_imgmsg(cv_image)
        compressed_image.header = msg.header
        # Publish compressed image
        self.publisher.publish(compressed_image)

def main(args=None):
    rclpy.init(args=args)
    image_compressor_node = ImageCompressorNode()
    rclpy.spin(image_compressor_node)
    image_compressor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
