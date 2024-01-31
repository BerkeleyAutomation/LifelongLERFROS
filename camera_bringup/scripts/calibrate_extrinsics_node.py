#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import message_filters

class ArucoMarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_marker_detection_node')
        self.realsense_subscription = message_filters.Subscriber(self, Image,'/repub_image_raw')
        self.arducam_subscription = message_filters.Subscriber(self, Image,'/repub/camLeft/image_raw')
        self.bridge = CvBridge()
        self.approx_sync = ApproximateTimeSynchronizer(
            [self.realsense_subscription, self.arducam_subscription],
            1,
            0.2,
        )
        self.approx_sync.registerCallback(self.image_callback)
        
        # Load ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

        # Create ArUco parameters
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.realsense_camera_matrix_ = np.array([[427.0931396484375,   0.  ,       422.375732421875],
                                        [  0.   , 426.6821594238281 ,236.3165283203125],
                                        [  0.      ,     0.   ,        1.        ]])
        
        self.realsense_distortion_coefficients_ = np.array([-0.05663116276264191, 0.06359340995550156 , -0.0006430497742258012, 0.0006550125544890761 , -0.020485159009695053])

        self.arducam_camera_matrix_ = np.array([[562.41060319 ,  0.      ,   332.43970168],
                              [ 0.    ,     561.87469896, 252.54629817],
                              [0.0, 0.0, 1.0]])
        
        self.arducam_distortion_coefficients_ = np.array([-4.12701786e-01 , 2.41486932e-01  ,6.50202348e-04, -4.13153471e-05,-9.32214031e-02])

    def image_callback(self, realsense_msg,arducam_msg):
        
        realsense_image = self.bridge.imgmsg_to_cv2(realsense_msg)
        arducam_image = self.bridge.imgmsg_to_cv2(arducam_msg)

        # Detect ArUco markers and estimate poses
        corners_realsense, ids_realsense, _ = cv2.aruco.detectMarkers(realsense_image, self.aruco_dict, parameters=self.aruco_params)
        corners_arducam, ids_arducam, _ = cv2.aruco.detectMarkers(arducam_image, self.aruco_dict, parameters=self.aruco_params)
        if ids_realsense is not None and ids_arducam is not None:
            rvecs_realsense, tvecs_realsense, _ = cv2.aruco.estimatePoseSingleMarkers(corners_realsense, markerLength=0.278, cameraMatrix=self.realsense_camera_matrix_, distCoeffs=self.realsense_distortion_coefficients_)
            rvecs_arducam, tvecs_arducam, _ = cv2.aruco.estimatePoseSingleMarkers(corners_arducam, markerLength=0.278, cameraMatrix=self.arducam_camera_matrix_, distCoeffs=self.arducam_distortion_coefficients_)
            rvec0_realsense = rvecs_realsense[0]
            rvec0_arducam = rvecs_arducam[0]
            tvec0_realsense = tvecs_realsense[0]
            tvec0_arducam = tvecs_arducam[0]

            realsense_to_aruco0_rotation,_ = cv2.Rodrigues(rvec0_realsense)
            arducam_to_aruco1_rotation,_ = cv2.Rodrigues(rvec0_arducam)

            realsense_to_aruco0_tf = np.eye(4)
            arducam_to_aruco1_tf = np.eye(4)
            realsense_to_aruco0_tf[:3, :3] = realsense_to_aruco0_rotation
            arducam_to_aruco1_tf[:3, :3] = arducam_to_aruco1_rotation

            realsense_to_aruco0_tf[:3, 3] = tvec0_realsense
            arducam_to_aruco1_tf[:3, 3] = tvec0_arducam
            
            aruco0_to_aruco1_tf = np.array([[ 0.99945048, -0.00374491, -0.03293514 ,-2.74821604],
 [ 0.0028986  , 0.99966533 ,-0.02570644, -0.04291193],
 [ 0.03302038 , 0.02559685,  0.99912685 ,-0.05436601],
 [ 0.         , 0.   ,       0.      ,    1.        ]])
            print("Realsense Tf: " + str(realsense_to_aruco0_tf))
            print("Arducam Tf 1: " + str(arducam_to_aruco1_tf))
            print("Realsense to Arducam: " + str(realsense_to_aruco0_tf @ aruco0_to_aruco1_tf @ np.linalg.inv(arducam_to_aruco1_tf)))
            # Draw marker poses on the image
            #for i in range(len(ids)):
            #    cv2.aruco.drawAxis(cv_image, cameraMatrix=self.camera_matrix_, distCoeffs=self.distortion_coefficients_, rvec=rvecs[i], tvec=tvecs[i], length=0.1)
            #    cv2.imwrite('axis_image.png',cv_image)
        #cv2.imshow('Aruco Marker Detection', realsense_image)
        #cv2.imshow('Arducam',arducam_image)
        #cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    aruco_marker_detection_node = ArucoMarkerDetectionNode()
    rclpy.spin(aruco_marker_detection_node)
    aruco_marker_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
