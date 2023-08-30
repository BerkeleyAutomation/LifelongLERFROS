#!/usr/bin/env python3


import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class NerfCollection(Node):
    def __init__(self):
        super().__init__('nerf_collection')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',  # Replace with your image topic name
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        except Exception as e:
            self.get_logger().error("Error converting image: %s" % str(e))
            return

        # Specify the folder where images will be saved
        output_folder = 'output_images'
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        # Save the image to the output folder
        image_filename = os.path.join(output_folder, f'image_{msg.header.stamp}.jpg')
        cv2.imwrite(image_filename, cv_image)
        self.get_logger().info(f'Saved image to {image_filename}')

def main(args=None):
    rclpy.init(args=args)
    nerf_collection = NerfCollection()
    rclpy.spin(nerf_collection)
    nerf_collection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
