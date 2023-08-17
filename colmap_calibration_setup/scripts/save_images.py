#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import os
import pathlib

class SaveImages(Node):
    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription_ = self.create_subscription(
            Image, "/input/image", self.imageCallback, 10
        )
        self.subscription_  # prevent unused variable warning
        self.i_ = 1
        self.bridge_ = CvBridge()
        self.photo_path_ = "lifelong_lerf_checkerboard_calibration_photos"
        self.declare_parameters(
            namespace='',
            parameters=[
                ('arg1',rclpy.Parameter.Type.INTEGER),
            ]
        )
        #arg1_param = self.get_parameter('arg1')
        #arg1 = arg1_param.value
        #self.i_ = int(arg1)

    def imageCallback(self, msg):
        try:
            cv_image = self.bridge_.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error("Error converting image: %s" % str(e))
            return
        home_path = str(pathlib.Path.home())
        directory_path = os.path.join(home_path,self.photo_path_)
        if not os.path.exists(directory_path):
            os.makedirs(directory_path)
        save_path = os.path.join(directory_path, "image_%s.png" % self.i_)

        try:
            cv2.imwrite(save_path, cv_image)
            self.get_logger().info("Image saved: %s" % save_path)
            self.i_ += 1
            #rclpy.shutdown()

        except Exception as e:
            self.get_logger().error("Error saving image: %s" % str(e))


def main(args=None):
    rclpy.init(args=args)

    save_image = SaveImages()

    rclpy.spin(save_image)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    save_image.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
