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
        self.image_sub = message_filters.Subscriber(self, Image, '/repub_image_raw')
        self.camera_info_sub = message_filters.Subscriber(self, CameraInfo, '/ros2_camera/color/camera_info')

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.camera_info_sub], 10, 0.1)
        self.ts.registerCallback(self.image_callback)
        self.bridge = CvBridge()

    def image_callback(self, image_msg, camera_info_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return
        
        # Detect ArUco markers
        markers = self.detect_markers(cv_image)
        print("running1")
        K = camera_info_msg.k
        self.camera_matrix = np.asarray(K).reshape((3,3))
        self.dist_coeff = np.asarray(camera_info_msg.d)

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # # left cam vals
        # self.camera_matrix = np.array([[569.04532018,0.,335.16605016],[0.,566.85332382, 243.60254152],[0.,           0.           ,1.]])
        # self.dist_coeff = np.array([[-4.30647187e-01 , 2.93329949e-01 , 5.02598200e-05, -2.25064466e-03,-1.50939072e-01]])

        # # right cam vals
        # self.camera_matrix = np.array([[562.42586682,   0.,         304.86531274],[0.         ,559.31246759 ,270.30609099],[  0.,           0.,           1.] ])
        # self.dist_coeff = ([[-4.22451798e-01 , 2.86381248e-01, -1.59815170e-04, -2.20573927e-03,-1.30771400e-01]])
        
        
        rvecs = [None, None]
        tvecs = [None, None] 
        # Print the pose information of detected markers
        for marker_id, corners in markers:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.278, self.camera_matrix, self.dist_coeff)
            if marker_id == 1:
                print("I see 1")
                rvecs[1] = rvec
                tvecs[1] = tvec
            if marker_id == 0:
                print("I see 0")
                rvec0, tvec0, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.278, self.camera_matrix, self.dist_coeff)
                rvecs[0] = rvec
                tvecs[0] = tvec
            
            cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeff, rvec, tvec, 0.1, thickness=5) 

            # Print the pose information
            self.get_logger().info(f'Detected ArUco marker with ID {marker_id}:')
            self.get_logger().info(f'Rotation Vector (rvec): {rvec}')
            self.get_logger().info(f'Translation Vector (tvec): {tvec}')
        
        if rvecs[1] is not None and rvecs[0] is not None:
            R_10, t_10 = self.calculate_relative_pose(rvecs[1], tvecs[1], rvecs[0], tvecs[0])
            print("relative rot between markers is ", R_10)
            print("relative trans between markers is ", t_10)
        
        rvecs = [None, None]
        tvecs = [None, None] 
        cv2.imshow("vis",cv_image)
        cv2.waitKey(0) 
  
        # closing all open windows 
        cv2.destroyAllWindows() 

    def detect_markers(self, image):
        # Your ArUco marker detection code using OpenCV
        # Example code:
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, _ = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

        if ids is not None:
            return zip(ids, corners)
        else:
            return []
    
    def calculate_relative_pose(self, rvec1, tvec1, rvec0, tvec0):
        # Convert rotation vectors to rotation matrices
        R1, _ = cv2.Rodrigues(rvec1)
        R0, _ = cv2.Rodrigues(rvec0)

        # Compute the relative rotation matrix
        R_10 = np.transpose(R1) @ R0

        # Calculate the relative translation vector
        t_10 = tvec0 - tvec1

        return R_10, t_10

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
