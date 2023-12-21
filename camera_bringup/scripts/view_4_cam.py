#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.topics = ['/repub/camFront/image_raw', '/repub/camBack/image_raw', '/repub/camLeft/image_raw', '/repub/camRight/image_raw', '/repub_image_raw']
        self.window_names = ['Front Camera', 'Back Camera', 'Left Camera', 'Right Camera']
        self.bridge = CvBridge()
        self.subs = {}
        self.windows = {}
        for topic_name in self.topics:
            sub = self.create_subscription(Image, topic_name, self.image_callback, 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow(msg.header.frame_id, cv_image)
        cv2.waitKey(1)  # Adjust the waitKey time based on your requirements

def main():
    rclpy.init()
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
