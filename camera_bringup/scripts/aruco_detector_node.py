#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class ArucoMarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_marker_detection_node')
        self.image_subscription = self.create_subscription(
            Image,
            '/repub/camRight/image_raw',  # Replace 'image_topic' with your actual image topic
            self.image_callback,
            10
        )
        self.image_subscription  # Prevent unused variable warning
        self.bridge = CvBridge()

        ### for left arducams
#         self.camera_matrix_ = np.array([[461.84353638   ,0.,         314.30229617],
#  [  0. ,        459.70959473, 231.27492743],
#  [  0.  ,         0.   ,        1.        ]])

#         self.distortion_coefficients_ = np.array([-4.20403295e-01, 1.10967255e-01, -9.18771396e-04, -2.37407887e-05,1.79913270e-01])


        # for right logitech cam
        # self.distortion_coefficients_ = np.array([[0.09048915, -0.53110363, -0.00777245, -0.00399641, 1.18761567]])

        # # New Camera Matrix
        # self.camera_matrix_ = np.array([[660.72302246, 0.0, 305.38355281],
        #                             [0.0, 659.38702393, 223.16278783],
        #                             [0.0, 0.0, 1.0]])
        # for left logitech cam
#         self.distortion_coefficients_ = np.array([[0.08549909, -0.36927898, -0.00902351, 0.0008513, 0.49198318]])

# # New Camera Matrix
#         self.camera_matrix_ = np.array([[645.22192383, 0.0, 314.02143305],
#                               [0.0, 649.07434082, 226.90158319],
#                               [0.0, 0.0, 1.0]])
        ### for realsense
        self.camera_matrix_ = np.array([[427.4412231,   0.  ,       422.37573242],
                                        [  0.   ,      427.0299377 ,236.31652832],
                                        [  0.      ,     0.   ,        1.        ]])
        self.distortion_coefficients_ = np.array([-0.05663116, 0.06359340, -0.000643049, 0.0006550125, -0.02048515])

        ### Good Left Arducam
        self.camera_matrix_ = np.array([[562.41060319,   0.      ,   332.43970168],
                                        [   0.    ,     561.87469896 ,252.54629817],
                                        [  0.    ,       0.      ,     1.        ]])
        self.distortion_coefficients_ = np.array([-4.12701786e-01 , 2.41486932e-01  ,6.50202348e-04 ,-4.13153471e-05,
  -9.32214031e-02])

        ### Good Right Arducam
        self.camera_matrix_ = np.array([[563.35316984 ,  0.       ,  292.44333299],
                                        [  0.   ,      562.47446886 ,250.12400081],
                                        [  0.    ,       0.      ,     1.        ]])
        self.distortion_coefficients_ = np.array([-4.06910715e-01 , 2.20253703e-01, -4.85489423e-05 , 6.24887100e-04,-7.10665449e-02])

        # Load ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

        # Create ArUco parameters
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Detect ArUco markers and estimate poses
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength=0.278, cameraMatrix=self.camera_matrix_, distCoeffs=self.distortion_coefficients_)
            rotation_matrix, _ = cv2.Rodrigues(rvecs)

            # Create a 4x4 transformation matrix
            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = tvecs.flatten()

            print("4x4 Transformation Matrix:")
            print(transform_matrix)
            # Draw marker poses on the image
            for i in range(len(ids)):
                cv2.aruco.drawAxis(cv_image, cameraMatrix=self.camera_matrix_, distCoeffs=self.distortion_coefficients_, rvec=rvecs[i], tvec=tvecs[i], length=0.1)

        cv2.imshow('Aruco Marker Detection', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_marker_detection_node = ArucoMarkerDetectionNode()
    rclpy.spin(aruco_marker_detection_node)
    aruco_marker_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
