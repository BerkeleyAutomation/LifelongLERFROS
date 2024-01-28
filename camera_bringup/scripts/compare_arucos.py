#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import matplotlib.pyplot as plt

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.cv_image = cv2.imread("/home/kushtimusprime/legs_ws/src/camera_bringup/scripts/IMG_5096.jpg")
        self.markers = self.detect_markers(self.cv_image)

        # self.camera_info_sub = self.create_subscription(CameraInfo,'/ros2_camera/color/camera_info', self.cam_info_cb, 1)

        self.camera_matrix = np.array([[3.36856975e+03, 0.00000000e+00, 1.46438425e+03],
 [0.00000000e+00, 3.37067117e+03, 2.02457507e+03],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) # Replace with camera matrix
        self.dist_coeff = np.array([[ 2.96988202e-01, -1.63242291e+00, -1.61121492e-03, -4.59038300e-03,
   3.08919992e+00]])  # Replace with distortion coefficients


        # K = camera_info_msg.k
        # self.camera_matrix = np.asarray(K).reshape((3,3))
        # self.dist_coeff = np.asarray(camera_info_msg.d)
        rvec1 = None
        tvec1 = None
        rvec0 = None
        tvec0 = None

        # Print the pose information of detected markers
        for marker_id, corners in self.markers:
            if marker_id == 1:
                print("I see 1")
                rvec1, tvec1, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.19, self.camera_matrix, self.dist_coeff)
            if marker_id == 0:
                print("I see 0")
                rvec0, tvec0, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.19, self.camera_matrix, self.dist_coeff)

            # Print the pose information
            self.get_logger().info(f'Detected ArUco marker with ID {marker_id}:')
            # self.get_logger().info(f'Rotation Vector (rvec): {rvec}')
            # self.get_logger().info(f'Translation Vector (tvec): {tvec}')
        if rvec1 is not None and rvec0 is not None:
            R_relative, t_relative = self.calculate_relative_pose(rvec1, tvec1, rvec0, tvec0)
            print("relative rot between markers is ", R_relative)
            print("relative trans between markers is ", t_relative)
        

    def detect_markers(self, image):
        # Your ArUco marker detection code using OpenCV
        # Example code:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

        if ids is not None:
            return zip(ids, corners)
        else:
            return []
    
    def calculate_relative_pose(self, rvec1, tvec1, rvec2, tvec2):
        # Convert rotation vectors to rotation matrices
        R1, _ = cv2.Rodrigues(rvec1)
        R2, _ = cv2.Rodrigues(rvec2)

        # Compute the relative rotation matrix
        R_relative = np.transpose(R1) @ R2

        # Calculate the relative translation vector
        t_relative = tvec2 - tvec1

        return R_relative, t_relative

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
