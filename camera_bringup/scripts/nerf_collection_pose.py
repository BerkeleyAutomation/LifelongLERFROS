#!/usr/bin/env python3


import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
import json
import numpy as np

class NerfPoseCollection(Node):
    def __init__(self):
        super().__init__('nerf_pose_collection')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',  # Replace with your image topic name
            self.image_callback,
            10)
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        self.bridge = CvBridge()
        self.image_counter = 0
        self.output_folder_ = 'output_images'
        self.json_file_path_ = os.path.join(self.output_folder_,'transforms.json')
        self.initial_tf_data_ = {
            "w": 1024,
            "h": 576,
            "fl_x": 421.45653935203393,
            "fl_y": 415.53758903338405,
            "cx": 521.1061090239227,
            "cy": 307.33126818633355,
            "k1": -0.024406477561223126,
            "k2": -0.0005270136831239189,
            "k3": -4.2935555001798215e-06,
            "k4": -0.00042943927562022905,
            "camera_model": "OPENCV_FISHEYE",
            "frames": []

        }
        with open(self.json_file_path_,"w") as write_file:
            json.dump(self.initial_tf_data_,write_file)

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
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        except Exception as e:
            self.get_logger().error("Error converting image: %s" % str(e))
            return

        # Specify the folder where images will be saved
        
        if not os.path.exists(self.output_folder_):
            os.makedirs(self.output_folder_)

        # Save the image to the output folder
        image_filename = os.path.join(self.output_folder_, f'image{self.image_counter:06d}.jpg')
        #print("IN HERE",flush=True)
        try:
            
            t = self.tf_buffer_.lookup_transform(from_frame,to_frame,msg.header.stamp)
            with open(self.json_file_path_,"r") as json_file:
                data = json.load(json_file)
            rotation = self.quaternion_rotation_matrix(t.transform.rotation)
            translation = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
            tf_matrix = np.column_stack((rotation,translation))
            tf_matrix = np.row_stack((tf_matrix,np.array([0,0,0,1])))
            frame_entry = {}
            frame_entry["file_path"] = f'image{self.image_counter:06d}.jpg'
            frame_entry["transform_matrix"] = tf_matrix.tolist()
            data["frames"].append(frame_entry)
            with open(self.json_file_path_,"w") as json_file:
                json.dump(data,json_file)

            cv2.imwrite(image_filename, cv_image)
            self.image_counter += 1
        except TransformException as ex:
            pass
            # try:
            #     t = self.tf_buffer_.lookup_transform(to_frame,from_frame,rclpy.time.Time())
            #     cv2.imwrite(image_filename, cv_image)
            #     self.image_counter += 1
            # except TransformException as ex:
            #     print("Tears pain sadness")
        #print(timestamp,flush=True)
        #print(type(timestamp),flush=True)
        #cv2.imwrite(image_filename, cv_image)
        #self.get_logger().info(f'Saved image to {image_filename}')

def main(args=None):
    rclpy.init(args=args)
    nerf_pose_collection = NerfPoseCollection()
    rclpy.spin(nerf_pose_collection)
    nerf_pose_collection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
