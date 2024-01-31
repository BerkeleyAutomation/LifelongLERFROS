#!/usr/bin/env python3
import cv2
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ChessboardDetectorNode(Node):
    def __init__(self):
        super().__init__('chessboard_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Replace 'image_topic' with your actual image topic
            self.image_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.bridge = CvBridge()
        self.image_count = 0
        self.save_folder = 'chessboard_images'

        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image_original = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        success, corners = self.detect_chessboard(cv_image)

        if success:
            cv2.drawChessboardCorners(cv_image, (8,5), corners, success)
            self.save_image(cv_image_original)

    def detect_chessboard(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (8,5), None)

        return ret, corners

    def save_image(self, image):
        self.image_count += 1
        image_filename = f'{self.save_folder}/image_{self.image_count:04d}.png'
        cv2.imwrite(image_filename, image)
        self.get_logger().info(f"Chessboard image saved: {image_filename}")

def main(args=None):
    rclpy.init(args=args)
    chessboard_detector_node = ChessboardDetectorNode()
    rclpy.spin(chessboard_detector_node)
    chessboard_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
