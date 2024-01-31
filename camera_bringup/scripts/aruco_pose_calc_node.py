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
            '/repub_image_raw',  # Replace 'image_topic' with your actual image topic
            self.image_callback,
            10
        )
        # self.image_subscription = self.create_subscription(
        #     Image,
        #     '/repub/camLeft/image_raw',  # Replace 'image_topic' with your actual image topic
        #     self.image_callback,
        #     10
        # )
        
        self.image_subscription  # Prevent unused variable warning
        self.bridge = CvBridge()
        # REALSENSE PARAMS
        self.camera_matrix_ = np.array([[427.0931396484375,   0.  ,       422.375732421875],
                                        [  0.   , 426.6821594238281 ,236.3165283203125],
                                        [  0.      ,     0.   ,        1.        ]])
        
        self.distortion_coefficients_ = np.array([-0.05663116276264191, 0.06359340995550156 , -0.0006430497742258012, 0.0006550125544890761 , -0.020485159009695053])

        # LEFT LOGITECH CAMS
        
        # self.camera_matrix_ = np.array([[645.22192383, 0.0, 314.02143305],
        #                       [0.0, 649.07434082, 226.90158319],
        #                       [0.0, 0.0, 1.0]])
        
        # self.distortion_coefficients_ = np.array([0.08549909, -0.36927898, -0.00902351, 0.0008513, 0.49198318])
        #self.camera_matrix_ = np.array([[596.0307352492937,   0.  ,       321.9236721296294],
        #                                [  0.   , 437.2343936031295 ,235.87197473612133],
        #                                [  0.      ,     0.   ,        1.        ]])
        
        #self.distortion_coefficients_ = np.array([-0.38581729554396127, 0.14310690556142414 , -0.001623479766128712, 0.0003939693438013992])
        h,w = 480,640
        #self.newcameramtx_, self.roi_ = cv2.getOptimalNewCameraMatrix(self.camera_matrix_, self.distortion_coefficients_, (w,h), 1, (w,h))
        # Load ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

        # Create ArUco parameters
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # cv_image = cv2.undistort(cv_image,self.camera_matrix_,self.distortion_coefficients_,None,self.newcameramtx_)
            # x,y,w,h = self.roi_
            # cv_image = cv_image[y:y+h,x:x+w]
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Detect ArUco markers and estimate poses
        corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength=0.278, cameraMatrix=self.camera_matrix_, distCoeffs=self.distortion_coefficients_)
            rvec0 = rvecs[0]
            rvec1 = rvecs[1]
            tvec0 = tvecs[0]
            tvec1 = tvecs[1]

            cam_to_aruco0_rotation,_ = cv2.Rodrigues(rvec0)
            cam_to_aruco1_rotation,_ = cv2.Rodrigues(rvec1)

            cam_to_aruco0_tf = np.eye(4)
            cam_to_aruco1_tf = np.eye(4)
            cam_to_aruco0_tf[:3, :3] = cam_to_aruco0_rotation
            cam_to_aruco1_tf[:3, :3] = cam_to_aruco1_rotation

            cam_to_aruco0_tf[:3, 3] = tvec0
            cam_to_aruco1_tf[:3, 3] = tvec1
            

            print("Aruco Tf 0: " + str(cam_to_aruco0_tf))
            print("Aruco Tf 1: " + str(cam_to_aruco1_tf))
            print("Aruco 0 to Aruco 1: " + str(np.linalg.inv(cam_to_aruco0_tf) @ cam_to_aruco1_tf))
            # Draw marker poses on the image
            for i in range(len(ids)):
                cv2.aruco.drawAxis(cv_image, cameraMatrix=self.camera_matrix_, distCoeffs=self.distortion_coefficients_, rvec=rvecs[i], tvec=tvecs[i], length=0.1)
                cv2.imwrite('axis_image.png',cv_image)
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
