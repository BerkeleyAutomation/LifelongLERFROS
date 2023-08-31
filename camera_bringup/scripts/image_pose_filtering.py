#!/usr/bin/env python3

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from lifelong_msgs.msg import ImagePose
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
import json
import numpy as np

class ImagePoseSubPub(Node):
    def __init__(self):
        super().__init__('image_pose_filtering')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',  # Replace with your image topic name
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            ImagePose, 
            '/camera/color/imagepose',
            10)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.bridge = CvBridge()
        self.image_counter = 0
        self.last_img_pos = None
        self.last_img_quat = None
        self.output_folder_ = 'output_images'
        self.json_file_path_ = os.path.join(self.output_folder_,'transforms.json')
        self.initial_tf_data_ = {
            "w": 424,
            "h": 240,
            "fl_x": 212.38006591796875,
            "fl_y": 212.1753387451172,
            "cx": 214.3612518310547,
            "cy": 120.91046142578125,
            "k1": -0.0553562305867672,
            "k2": 0.0682036280632019,
            "k3": -0.022622255608439445,
            "camera_model": "OPENCV",
            "frames": []

        }

    def quaternion_rotation_matrix(self,Q):
        # Extract the values from Q
        q0 = Q.w
        q1 = Q.x
        q2 = Q.y
        q3 = Q.z
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix 
    def image_callback(self, msg):
        from_frame = 'map'
        to_frame = 'camera'
        fixed_frame = 'odom'

        pub_msg = ImagePose()

        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        except Exception as e:
            self.get_logger().error("Error converting image: %s" % str(e))
            return
        
        try:
            t = self.tf_buffer_.lookup_transform(from_frame,to_frame,rclpy.time.Time())
            quaternion = np.array([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w])
            rotation = self.quaternion_rotation_matrix(t.transform.rotation)
            translation = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
            tf_matrix = np.column_stack((rotation,translation))
            tf_matrix = np.row_stack((tf_matrix,np.array([0,0,0,1])))

            self.image_counter += 1
            print(self.image_counter,flush=True)
        except TransformException as ex:
            print("Yeah, I suck",flush=True)
            print(ex)
            return

        # Filter images
        pose_to_pub = Pose()
        if self.last_img_pos is not None:
            rot_dist = 1 - np.dot(self.last_img_quat, quaternion) ** 2
            pos_dist = np.linalg.norm(self.last_img_pos - translation)
            dist = 0.5 * rot_dist + 0.5 * pos_dist
            if dist < .05:
                #exit 
                return 

        self.last_img_pos = translation
        self.last_img_quat = quaternion

        pose_to_pub.position.x = translation[0]
        pose_to_pub.position.y = translation[1]
        pose_to_pub.position.z = translation[2]
        pose_to_pub.orientation.x = quaternion[0]
        pose_to_pub.orientation.y = quaternion[1]
        pose_to_pub.orientation.z = quaternion[2]
        pose_to_pub.orientation.w = quaternion[3]

        pub_msg.pose = pose_to_pub
        pub_msg.img = self.bridge.cv2_to_imgmsg(cv_image)
        pub_msg.w = self.initial_tf_data_['w']
        pub_msg.h = self.initial_tf_data_['h']
        pub_msg.fl_x = self.initial_tf_data_['fl_x']
        pub_msg.fl_y = self.initial_tf_data_['fl_y']
        pub_msg.cx = self.initial_tf_data_['cx']
        pub_msg.cy = self.initial_tf_data_['cy']
        pub_msg.k1 = self.initial_tf_data_['k1']
        pub_msg.k2 = self.initial_tf_data_['k2']
        pub_msg.k3 = self.initial_tf_data_['k3']
        
        self.publisher.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    img_pose = ImagePoseSubPub()
    rclpy.spin(img_pose)
    img_pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
