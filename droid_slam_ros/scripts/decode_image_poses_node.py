#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import os
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Bool
import glob
from geometry_msgs.msg import Pose,Point,Quaternion
import matplotlib.pyplot as plt

from lifelong_msgs.msg import ImagePose, ImagePoses
from geometry_msgs.msg import TransformStamped
import cv2
import time
from cv_bridge import CvBridge

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.cv_bridge_ = CvBridge()
        self.sim_realsense_sub = self.create_subscription(ImagePoses,'/camera/color/imagepose',self.image_poses_cb,1)
        self.right_pub = self.create_publisher(Image, 'cam_right',1)
        self.rs_pub = self.create_publisher(Image, 'cam_rs',1)
        self.left_pub = self.create_publisher(Image, 'cam_left',1)
        self.i = 0
        self.left_dir = "left_imgs"
        self.right_dir = "right_imgs"
        self.rs_dir ="rs_imgs"
        if not os.path.exists(self.left_dir):
            os.makedirs(self.left_dir)
        if not os.path.exists(self.right_dir):
            os.makedirs(self.right_dir)
        if not os.path.exists(self.rs_dir):
            os.makedirs(self.rs_dir)

    def image_poses_cb(self,image_poses):
        print("in the callback")
        rs_imgmsg = image_poses.image_poses[0].img
        left_imgmsg = image_poses.image_poses[1].img
        right_imgmsg = image_poses.image_poses[2].img
        filename = f"image{str(self.i).zfill(4)}.jpg"
        rs_path = os.path.join(self.rs_dir, filename)
        left_path = os.path.join(self.left_dir, filename)
        right_path = os.path.join(self.right_dir, filename)
        rs_img = self.cv_bridge_.compressed_imgmsg_to_cv2(rs_imgmsg)
        right_img = self.cv_bridge_.compressed_imgmsg_to_cv2(right_imgmsg)
        left_img = self.cv_bridge_.compressed_imgmsg_to_cv2(left_imgmsg)
        self.rs_pub.publish(self.cv_bridge_.cv2_to_imgmsg(rs_img))
        self.right_pub.publish(self.cv_bridge_.cv2_to_imgmsg(right_img))
        self.left_pub.publish(self.cv_bridge_.cv2_to_imgmsg(left_img))
        cv2.imwrite(rs_path, rs_img)
        cv2.imwrite(left_path, left_img)
        cv2.imwrite(right_path, right_img)
        self.i +=1

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()