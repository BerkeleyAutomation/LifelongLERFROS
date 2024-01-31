# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo

import os
import glob 
import cv2
import numpy as np
from cv_bridge import CvBridge
import torch
import torch.nn.functional as F
from tqdm import tqdm
import time


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # self.depth_publisher = self.create_publisher(String, 'topic', 10)
        self.depth_publisher = self.create_publisher(Image, '/repub_depth_raw', 10)
        self.rgb_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/ros2_camera/color/camera_info', 10)
        self.loop_done_publisher = self.create_publisher(String, '/loop_done', 10)
        timer_period = 2 # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        which_dataset = 'large_loop_1'
        self.depth_path = f'./src/droid_slam_ros/datasets/ETH3D-SLAM/training/{which_dataset}/depth'
        self.depth_filenames = sorted(os.listdir(self.depth_path))
        self.rgb_path = f'./src/droid_slam_ros/datasets/ETH3D-SLAM/training/{which_dataset}/rgb'
        self.rgb_filenames = sorted(os.listdir(self.rgb_path))
        self.bridge = CvBridge()
        self.timer_callback()
        import pdb
        pdb.set_trace()

    def image_stream(self, datapath, use_depth=False, stride=1):
        """ image generator """

        fx, fy, cx, cy = np.loadtxt(os.path.join(datapath, 'calibration.txt')).tolist()
        image_list = sorted(glob.glob(os.path.join(datapath, 'rgb', '*.png')))[::stride]
        depth_list = sorted(glob.glob(os.path.join(datapath, 'depth', '*.png')))[::stride]

        for t, (image_file, depth_file) in enumerate(zip(image_list, depth_list)):
            image = cv2.imread(image_file)
            depth = cv2.imread(depth_file, cv2.IMREAD_ANYDEPTH) / 5000.0

            h0, w0, _ = image.shape
            h1 = int(h0 * np.sqrt((384 * 512) / (h0 * w0)))
            w1 = int(w0 * np.sqrt((384 * 512) / (h0 * w0)))

            image = cv2.resize(image, (w1, h1))
            image = image[:h1-h1%8, :w1-w1%8]
            # image = torch.as_tensor(image).permute(2, 0, 1)
            
            # depth = torch.as_tensor(depth)
            # depth = F.interpolate(depth[None,None], (h1, w1)).squeeze()
            depth = depth[:h1-h1%8, :w1-w1%8]

            intrinsics = torch.as_tensor([fx, fy, cx, cy])
            intrinsics[0::2] *= (w1 / w0)
            intrinsics[1::2] *= (h1 / h0)
            
            if use_depth:
                # yield t, image[None], depth, intrinsics
                yield t, image, depth, intrinsics

            else:
                # yield t, image[None], intrinsics
                yield t, image, intrinsics


    def timer_callback(self):
        # depth_img = cv2.imread(f'{self.depth_path}/{self.depth_filenames[self.i]}')     
        # depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)
        # depth_img_msg = self.bridge.cv2_to_imgmsg(np.array(depth_img, dtype=np.uint16), "16UC1")
        # depth_img_msg.header.stamp = self.get_clock().now().to_msg()
        # # depth_img_msg.data = self.bridge.cv2_to_imgmsg(np.array(depth_img, dtype=np.uint16), "16UC1"),
        # # depth_img_msg.data = (np.array(depth_img, dtype=np.uint8))
        # self.depth_publisher.publish(depth_img_msg)

        # # self.depth_publisher.publish(depth_img)
        # rgb_img = cv2.imread(f'{self.rgb_path}/{self.rgb_filenames[self.i]}')
        # # rgb_img_msg = Image()
        # rgb_img_msg = self.bridge.cv2_to_imgmsg(np.array(rgb_img), "bgr8")
        # rgb_img_msg.header.stamp = self.get_clock().now().to_msg()
        # self.rgb_publisher.publish(rgb_img_msg)
        # self.get_logger().info(f'Publishing: {self.i}')

        # cam_info_msg = CameraInfo()
        # cam_info_msg.header.frame_id = 'map_droid'
        # cam_info_msg.distortion_model = 'plumb_bob'
        # cam_info_msg.k = np.array([[535.4, 0, 320.1], [0, 539.2, 247.6], [0, 0, 1]], dtype=np.float64)
        # cam_info_msg.d = np.array([0, 0, 0, 0, 0], dtype=np.float64).tolist()
        # self.camera_info_publisher.publish(cam_info_msg)

        ################################################################################

        datapath = './src/droid_slam_ros/datasets/ETH3D-SLAM/training/sfm_house_loop'
        stride = 1
        
        for (t, image, depth, intrinsics) in tqdm(self.image_stream(datapath, use_depth=True, stride=stride)):
            intrinsics = torch.as_tensor([intrinsics[0], 0, intrinsics[2], 0, intrinsics[1], intrinsics[3], 0, 0, 1])
            intrinsics = intrinsics.cpu().numpy().astype(np.float64)
            # intrinsics = [intrinsics[0], 0, intrinsics[2], 0, intrinsics[1], intrinsics[3], 0, 0, 1]

            depth_img_msg = self.bridge.cv2_to_imgmsg(np.array(depth, dtype=np.uint16), "16UC1")
            depth_img_msg.header.stamp = self.get_clock().now().to_msg()

            rgb_img_msg = self.bridge.cv2_to_imgmsg(np.array(image), "bgr8")
            rgb_img_msg.header.stamp = self.get_clock().now().to_msg()

            cam_info_msg = CameraInfo()
            cam_info_msg.header.frame_id = 'map_droid'
            cam_info_msg.header.stamp = self.get_clock().now().to_msg()
            cam_info_msg.distortion_model = 'plumb_bob'
            cam_info_msg.k = intrinsics
            cam_info_msg.d = np.array([0, 0, 0, 0, 0], dtype=np.float64).tolist()
            cam_info_msg.height, cam_info_msg.width = image.shape[0], image.shape[1]

            self.depth_publisher.publish(depth_img_msg)
            self.rgb_publisher.publish(rgb_img_msg)
            self.camera_info_publisher.publish(cam_info_msg)
            time.sleep(0.1)
            

        msg = String()
        msg.data = 'done'
        self.loop_done_publisher.publish(msg)
        self.get_logger().info('loop done')
        

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
