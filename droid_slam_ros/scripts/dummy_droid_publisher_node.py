#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import os
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
import glob
import cv2
import time
from cv_bridge import CvBridge

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.rgb_publisher_ = self.create_publisher(Image, '/repub_image_raw', 10)
        self.depth_publisher_ = self.create_publisher(Image, '/repub_depth_raw', 10)
        self.camera_info_publisher_ = self.create_publisher(CameraInfo, '/ros2_camera/color/camera_info', 10)
        self.done_publisher_ = self.create_publisher(Bool,'/loop_done',10)
        self.cv_bridge_ = CvBridge()
        datapath = "/home/kushtimusprime/legs_ws/src/droid_slam_ros/datasets/ETH3D-SLAM/training/sfm_house_loop"
        stride = 1
        fx, fy, cx, cy = np.loadtxt(os.path.join(datapath, 'calibration.txt')).tolist()
        image_list = sorted(glob.glob(os.path.join(datapath, 'rgb', '*.png')))[::stride]
        depth_list = sorted(glob.glob(os.path.join(datapath, 'depth', '*.png')))[::stride]

        for t, (image_file, depth_file) in enumerate(zip(image_list, depth_list)):
            image = cv2.imread(image_file)
            depth = cv2.imread(depth_file, cv2.IMREAD_ANYDEPTH) / 5000.0
            ros_image = self.cv_bridge_.cv2_to_imgmsg(image,encoding="passthrough")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_depth = self.cv_bridge_.cv2_to_imgmsg(depth,encoding="passthrough")
            ros_depth.header.stamp = ros_image.header.stamp
            ros_camera_info = CameraInfo()
            ros_camera_info.header.stamp = ros_image.header.stamp
            ros_camera_info.k = [fx,0.0,cx,0.0,fy,cy,0.0,0.0,1.0]
            self.rgb_publisher_.publish(ros_image)
            self.depth_publisher_.publish(ros_depth)
            self.camera_info_publisher_.publish(ros_camera_info)
            time.sleep(0.2)
        bool_msg = Bool()
        bool_msg.data = True
        self.done_publisher_.publish(bool_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()