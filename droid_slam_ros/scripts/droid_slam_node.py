#!/usr/bin/env python3
from sensor_msgs.msg import Image, CompressedImage,CameraInfo
from geometry_msgs.msg import Pose,Point,Quaternion
from lifelong_msgs.msg import ImagePose  # Make sure to import your custom ImagePose message
from rclpy.node import Node
from cv_bridge import CvBridge  # Needed for converting between ROS Image messages and OpenCV images
import sys
from scipy.spatial.transform import Rotation as R
import os
file_location = os.path.dirname(os.path.realpath(__file__))
sys.path.append(file_location+'/../../share/droid_slam_ros/droid_slam')
from PIL import Image

from tqdm import tqdm
import numpy as np
import torch
import lietorch
import cv2
import glob 
import time
import argparse
import rclpy
import json
from sensor_msgs.msg import Image as ROSImage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from torch.multiprocessing import Process
from droid import Droid
import message_filters
from tf2_ros import TransformBroadcaster

import torch.nn.functional as F
import droid_backends
from lietorch import SE3

class DroidNode(Node):
    def __init__(self, args):
        super().__init__('droid_node')
        self.droid = None
        self.args = args
        self.cam_transform = np.diag([1, -1, -1, 1])
        # Initialize ROS2 Publisher and Subscriber
        self.publisher = self.create_publisher(ImagePose, '/camera/color/imagepose',10)
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, '/camera/pointcloud', 10)
        self.realsense_publisher = self.create_publisher(ImagePose, '/sim_realsense',20)
        # self.realsense_subscriber = self.create_subscription(
        #     ImagePose,
        #     '/sim_realsense',
        #     self.listener_callback,
        #     10)
        self.rgb_sub = message_filters.Subscriber(self,
            CompressedImage,
            '/imageo_compressedo'#/camera/color/image_raw/compressed', #'/imageo_compressedo',
            )
        self.intr_sub = self.create_subscription(CameraInfo,'/ros2_camera/color/camera_info',self.cam_intr_cb,1)
        self.sim_realsense_sub = self.create_subscription(ImagePose,'/sim_realsense',self.sim_realsense_callback,10)
        self.depth_sub = message_filters.Subscriber(self,ROSImage,'/ros2_camera/depth/image_rect_raw')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 20, 0.1)
        self.ts.registerCallback(self.image_callback)

        self.image_counter = 0
        self.output_folder_ = 'output_images'
        self.json_file_path_ = os.path.join(self.output_folder_,'transforms.json')
        if not os.path.exists(self.output_folder_):
            # Create the directory
            os.makedirs(self.output_folder_)
        self.cam_params = {

        }
        self.bridge = CvBridge()
        self.last_pose = None
        self.last_disp = None

    def cam_intr_cb(self,msg):
        if 'w' in self.cam_params:
            return
        k1 = msg.d[0]
        k2 = msg.d[1]
        k3 = msg.d[4]
        K = msg.k.reshape(3,3)
        params = {
            "w": msg.width,
            "h": msg.height,
            "fl_x": K[0,0],
            "fl_y": K[1,1],
            "cx": K[0,2],
            "cy": K[1,2],
            "k1": k1,
            "k2": k2,
            "k3": k3,
            "camera_model": "OPENCV",
        }
        self.frames = []
        self.cam_params |= params

    def xyzquat2mat(self,vec):
        xyz = vec[:3]
        quat = vec[3:]
        matrix = np.eye(4)
        rotation = R.from_quat(quat)
        matrix[:3, :3] = rotation.as_matrix()
        matrix[:3, 3] = xyz
        matrix = np.linalg.inv(matrix) @ self.cam_transform
        return matrix

    def sim_realsense_callback(self,msg):
        print("sim realsense callback",self.image_counter)
        cv_image_original = self.bridge.imgmsg_to_cv2(msg.img, desired_encoding='bgr8')  # Convert ROS Image message to OpenCV image
        cv_image = cv2.resize(cv_image_original, (848, 480))
        depth_image_original = self.bridge.imgmsg_to_cv2(msg.depth,desired_encoding='16UC1')
        depth_image = cv2.resize(depth_image_original, (848, 480))
        if 'cx' not in self.cam_params:
            print("Not recieved intr yet, skipping frame")
            return
        t = msg.img.header.stamp.sec  # Current ROS time
        depth_tensor = torch.from_numpy(depth_image.astype(np.int32)).float() * .001 #one mm per unit, convert to meters
        # Replace this part with how you get your intrinsics
        intrinsics = torch.as_tensor([self.cam_params['fl_x'],self.cam_params['fl_y'],self.cam_params['cx'],self.cam_params['cy']])

        if self.droid is None:
            self.args.image_size = [cv_image.shape[0], cv_image.shape[1]]
            print('img size',self.args.image_size)
            self.droid = Droid(self.args)
        
        image_tensor = torch.as_tensor(cv_image).permute(2, 0, 1)
        # import pdb; pdb.set_trace()
        # print(image_tensor.shape)
        # print(depth_tensor.shape)
        # only add the first 3 depth images to get a scene scale, then ignore them since it messes
        # up tracking sometimes
        if self.image_counter>3:
            depth=None
        else:
            depth=depth_tensor[:,:]
        self.droid.track(t, image_tensor[None, :, :, :],depth, intrinsics=intrinsics)
        #visualize the image with cv2
        if(self.droid.video.counter.value == self.image_counter):
            if(self.last_pose is not None):
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "map"
                t.child_frame_id = "base_footprint"
                
                last_pose_arr = [self.last_pose.position.x,self.last_pose.position.y,self.last_pose.position.z,self.last_pose.orientation.x,self.last_pose.orientation.y,self.last_pose.orientation.z,self.last_pose.orientation.w]
                xyz = last_pose_arr[:3]
                quat = last_pose_arr[3:]
                matrix = np.eye(4)
                rotation = R.from_quat(quat)
                matrix[:3, :3] = rotation.as_matrix()
                matrix[:3, 3] = xyz
                droid_slam_alt_tf = np.array([[0,0,1,0],[-1,0,0,0],[0,-1,0,0],[0,0,0,1]])
                map_to_ros_cam = droid_slam_alt_tf @ matrix
                ros_cam_to_base = np.array([[0,-1,0,0],[0,0,1,0],[-1,0,0,0],[0,0,0,1]])
                map_to_base = map_to_ros_cam @ ros_cam_to_base
                alt_position = map_to_base[:3,3]
                alt_rotation_matrix = map_to_base[:3,:3]
                alt_quaternion = R.from_matrix(alt_rotation_matrix).as_quat()
                t.transform.translation.x = alt_position[0]
                t.transform.translation.y = alt_position[1]
                t.transform.translation.z = alt_position[2]

                t.transform.rotation.x = alt_quaternion[0]
                t.transform.rotation.y = alt_quaternion[1]
                t.transform.rotation.z = alt_quaternion[2]
                t.transform.rotation.w = alt_quaternion[3]

                self.tf_broadcaster.sendTransform(t)
                print("Sent transform",flush=True)
                # Make it so dirty_index is a tensor but it just includes the last one
                # Then you should get a [1,60,106,3] pointcloud which is just [60,106] pointcloud
                dirty_index = torch.where(self.droid.video.dirty.clone())[0]
                if(len(dirty_index) > 0):
                    dirty_index = dirty_index[-1]
                    poses = torch.index_select(self.droid.video.poses,0,dirty_index)
                    disps = torch.index_select(self.droid.video.disps,0,dirty_index)
                    points = droid_backends.iproj(SE3(poses).inv().data, disps, self.droid.video.intrinsics[0]).cpu()
                    points = points.reshape(-1,3).numpy()
                    pointcloud_msg = PointCloud2()
                    pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
                    pointcloud_msg.header.frame_id = "ros2_pointcloud"
                    pointcloud_msg.height = 1
                    pointcloud_msg.width = len(points)
                    pointcloud_msg.fields = [
                        PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
                        PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
                        PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1)
                    ]
                    pointcloud_msg.is_bigendian = False
                    pointcloud_msg.point_step = 12
                    pointcloud_msg.row_step = 12 * len(points)
                    pointcloud_msg.is_dense = False
                    pointcloud_msg.data = points.astype(np.float32).tobytes()
                    self.pointcloud_publisher_.publish(pointcloud_msg)
                    
            return
        #imshow the cv_image
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        self.image_counter += 1
        pose = self.droid.video.poses[self.droid.video.counter.value-1].cpu().numpy()
        
        print("Adding droid keyframe...")
        image_pose_msg = ImagePose()
        image_pose_msg.img = self.bridge.cv2_to_imgmsg(cv_image_original, encoding="bgr8")
        image_pose_msg.depth = self.bridge.cv2_to_imgmsg(depth_image_original,encoding="16UC1")
        image_pose_msg.w = self.cam_params['w']
        image_pose_msg.h = self.cam_params['h']
        image_pose_msg.fl_x = self.cam_params['fl_x']
        image_pose_msg.fl_y = self.cam_params['fl_y']
        image_pose_msg.cx = self.cam_params['cx']
        image_pose_msg.cy = self.cam_params['cy']
        image_pose_msg.k1 = self.cam_params['k1']
        image_pose_msg.k2 = self.cam_params['k2']
        image_pose_msg.k3 = self.cam_params['k3']
        posemat = self.xyzquat2mat(pose)
        pose = posemat[:3,3]
        print("xyz pose droidslam: ", pose)
        orient = R.from_matrix(posemat[:3,:3]).as_quat()
        image_pose_msg.pose = Pose(position=Point(x=pose[0],y=pose[1],z=pose[2]),orientation=Quaternion(x=orient[0],y=orient[1],z=orient[2],w=orient[3]))  # Replace with your Pose message
        self.last_pose = image_pose_msg.pose
        self.last_disp = self.droid.video.disps[self.droid.video.counter.value-1].cpu().numpy()

        self.publisher.publish(image_pose_msg)
        filename = f"{self.output_folder_}/image{self.image_counter:06d}.jpg"
        Image.fromarray(image_tensor.squeeze().cpu().permute(1,2,0).numpy()[:,:,::-1].astype(np.uint8)).save(filename)
        frame_dat = {'transform_matrix':posemat[:3,:].tolist(),'file_path':filename}
        self.frames.append(frame_dat)        

    def image_callback(self, img_msg,depth_msg):
        realsense_sim_msg = ImagePose()
        cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding='bgr8')  # Convert ROS Image message to OpenCV image
        cv_img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        realsense_sim_msg.img = cv_img_msg
        realsense_sim_msg.depth = depth_msg
        self.realsense_publisher.publish(realsense_sim_msg)
        return
        if 'cx' not in self.cam_params:
            print("Not recieved intr yet, skipping frame")
            return
        t = img_msg.header.stamp.sec  # Current ROS time
        cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding='bgr8')  # Convert ROS Image message to OpenCV image
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg,desired_encoding='16UC1')
        realsense_sim_msg = ImagePose()
        realsense_sim_msg.img = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        realsense_sim_msg.depth = self.bridge.cv2_to_imgmsg(depth_image,encoding="16UC1")
        self.realsense_publisher.publish(realsense_sim_msg)
        depth_tensor = torch.from_numpy(depth_image.astype(np.int32)).float() * .001 #one mm per unit, convert to meters
        # Replace this part with how you get your intrinsics
        intrinsics = torch.as_tensor([self.cam_params['fl_x'],self.cam_params['fl_y'],self.cam_params['cx'],self.cam_params['cy']])

        if self.droid is None:
            self.args.image_size = [cv_image.shape[0], cv_image.shape[1]]
            print('img size',self.args.image_size)
            self.droid = Droid(self.args)
        
        image_tensor = torch.as_tensor(cv_image).permute(2, 0, 1)
        # import pdb; pdb.set_trace()
        # print(image_tensor.shape)
        # print(depth_tensor.shape)
        # only add the first 3 depth images to get a scene scale, then ignore them since it messes
        # up tracking sometimes
        if self.image_counter>3:
            depth=None
        else:
            depth=depth_tensor[:,:]
        self.droid.track(t, image_tensor[None, :, :, :],depth, intrinsics=intrinsics)

        if(self.droid.video.counter.value == self.image_counter):
            return
        self.image_counter += 1
        pose = self.droid.video.poses[self.droid.video.counter.value-1].cpu().numpy()
        print("Adding droid keyframe...")
        image_pose_msg = ImagePose()
        image_pose_msg.img = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        image_pose_msg.depth = self.bridge.cv2_to_imgmsg(depth_image,encoding="16UC1")
        self.realsense_publisher.publish(image_pose_msg)
        image_pose_msg.w = self.cam_params['w']
        image_pose_msg.h = self.cam_params['h']
        image_pose_msg.fl_x = self.cam_params['fl_x']
        image_pose_msg.fl_y = self.cam_params['fl_y']
        image_pose_msg.cx = self.cam_params['cx']
        image_pose_msg.cy = self.cam_params['cy']
        image_pose_msg.k1 = self.cam_params['k1']
        image_pose_msg.k2 = self.cam_params['k2']
        image_pose_msg.k3 = self.cam_params['k3']
        posemat = self.xyzquat2mat(pose)
        pose = posemat[:3,3]
        print("xyz pose droidslam: ", pose)
        orient = R.from_matrix(posemat[:3,:3]).as_quat()
        image_pose_msg.pose = Pose(position=Point(x=pose[0],y=pose[1],z=pose[2]),orientation=Quaternion(x=orient[0],y=orient[1],z=orient[2],w=orient[3]))  # Replace with your Pose message
        
        self.publisher.publish(image_pose_msg)
        filename = f"{self.output_folder_}/image{self.image_counter:06d}.jpg"
        Image.fromarray(image_tensor.squeeze().cpu().permute(1,2,0).numpy()[:,:,::-1].astype(np.uint8)).save(filename)
        frame_dat = {'transform_matrix':posemat[:3,:].tolist(),'file_path':filename}
        self.frames.append(frame_dat)

    def saveJSON(self):
        print("In here")
        self.cam_params['frames']=self.frames
        with open(self.json_file_path_,"w") as json_file:
            json.dump(self.cam_params,json_file)
        print("Outta here")

def main(mainargs=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--imagedir",type=str, help="path to image directory")
    parser.add_argument("--calib", type=str, help="path to calibration file")
    parser.add_argument("--t0", default=0, type=int, help="starting frame")
    parser.add_argument("--stride", default=1, type=int, help="frame stride")
    file_location = os.path.dirname(os.path.realpath(__file__))
    parser.add_argument("--weights", default=file_location + "/../../share/droid_slam_ros/droid.pth")
    parser.add_argument("--buffer", type=int, default=512)
    parser.add_argument("--image_size", default=[240, 320])
    parser.add_argument("--disable_vis", action="store_true")

    parser.add_argument("--beta", type=float, default=0.3, help="weight for translation / rotation components of flow")
    parser.add_argument("--filter_thresh", type=float, default=2, help="how much motion before considering new keyframe")
    parser.add_argument("--warmup", type=int, default=4, help="number of warmup frames")
    parser.add_argument("--keyframe_thresh", type=float, default=3, help="threshold to create a new keyframe")
    parser.add_argument("--frontend_thresh", type=float, default=16.0, help="add edges between frames whithin this distance")
    parser.add_argument("--frontend_window", type=int, default=50, help="frontend optimization window")
    parser.add_argument("--frontend_radius", type=int, default=2, help="force edges between frames within radius")
    parser.add_argument("--frontend_nms", type=int, default=1, help="non-maximal supression of edges")

    parser.add_argument("--backend_thresh", type=float, default=22.0)
    parser.add_argument("--backend_radius", type=int, default=2)
    parser.add_argument("--backend_nms", type=int, default=3)
    parser.add_argument("--upsample", action="store_true")
    parser.add_argument("--reconstruction_path", help="path to saved reconstruction")
    args,_ = parser.parse_known_args()
    args.stereo = False

    torch.multiprocessing.set_start_method('spawn')
    rclpy.init(args=mainargs)

    # Parse the arguments as before
    # ...
    node = DroidNode(args)
    try:
        rclpy.spin(node)  # Keep the node alive
    except KeyboardInterrupt:
        node.saveJSON()
        print("Exiting...")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
