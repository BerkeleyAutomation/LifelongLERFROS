#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import os
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
import glob
from geometry_msgs.msg import Pose,Point,Quaternion

from lifelong_msgs.msg import ImagePose
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import cv2
import time
from cv_bridge import CvBridge
import argparse
import torch
import message_filters
from tf2_ros import TransformBroadcaster
import sys
file_location = os.path.dirname(os.path.realpath(__file__))
sys.path.append(file_location+'/../../share/droid_slam_ros/droid_slam')
from droid import Droid

import torch.nn.functional as F

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.cv_bridge_ = CvBridge()
        self.publisher = self.create_publisher(ImagePose, '/camera/color/imagepose',10)
        self.cam_params = {}
        self.rgb_sub = message_filters.Subscriber(self, Image, '/repub_image_raw')
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, '/ros2_camera/color/camera_info')
        self.intr_sub = self.create_subscription(CameraInfo,'/ros2_camera/color/camera_info',self.cam_intr_cb,1)
        self.depth_sub = message_filters.Subscriber(self,Image,'/ros2_camera/depth/image_rect_raw')
        self.done_sub_ = self.create_subscription(Bool,'/loop_done',self.doneCallback,10)
        self.realsense_publisher = self.create_publisher(ImagePose, '/sim_realsense',1)
        self.tstamps = []
        self.tf_broadcaster = TransformBroadcaster(self)
        self.last_pose = None
        self.t_ = 0
        self.iter_times = []
        self.image_counter = 0
        self.stride = 1
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 1, 0.5)
        self.ts.registerCallback(self.image_callback_uncompressed)
        #self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.cam_info_sub], 10, 0.2)
        #self.ts.registerCallback(self.fullImageCallback)
        parser = argparse.ArgumentParser()
        parser.add_argument("--datapath",default="/home/kushtimusprime/legs_ws/src/droid_slam_ros/datasets/ETH3D-SLAM/training/sfm_house_loop")
        parser.add_argument("--weights", default="droid.pth")
        parser.add_argument("--buffer", type=int, default=1024)
        parser.add_argument("--image_size", default=[240, 320])
        parser.add_argument("--disable_vis", action="store_true")
        parser.add_argument("--upsample", action="store_true")
        parser.add_argument("--beta", type=float, default=0.5)
        parser.add_argument("--filter_thresh", type=float, default=2.0)
        parser.add_argument("--warmup", type=int, default=8)
        parser.add_argument("--keyframe_thresh", type=float, default=3.5)
        parser.add_argument("--frontend_thresh", type=float, default=16.0)
        parser.add_argument("--frontend_window", type=int, default=16)
        parser.add_argument("--frontend_radius", type=int, default=1)
        parser.add_argument("--frontend_nms", type=int, default=0)

        parser.add_argument("--stereo", action="store_true")
        parser.add_argument("--depth", action="store_false",default=False)

        parser.add_argument("--backend_thresh", type=float, default=22.0)
        parser.add_argument("--backend_radius", type=int, default=2)
        parser.add_argument("--backend_nms", type=int, default=3)

        self.droid_args_ = parser.parse_args()
        torch.multiprocessing.set_start_method('spawn')
        print("Running evaluation on {}".format(self.droid_args_.datapath))
        print(self.droid_args_)

        self.cam_transform = np.diag([1, -1, -1, 1])
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 20, 0.5)
        self.ts.registerCallback(self.image_callback_uncompressed)
        self.sim_realsense_sub = self.create_subscription(ImagePose,'/sim_realsense',self.sim_realsense_callback,1)

    def xyzquat2mat(self,vec):
        print('vec:', vec)
        xyz = vec[:3]
        quat = vec[3:]
        matrix = np.eye(4)
        try:
            rotation = R.from_quat(quat)
        except ValueError:
            print('quat:', quat)
            import pdb; pdb.set_trace()
        # rotation = R.from_quat(quat[3] + quat[:4])
        matrix[:3, :3] = rotation.as_matrix()
        matrix[:3, 3] = xyz
        matrix = np.linalg.inv(matrix) @ self.cam_transform
        return matrix
    
    def sim_realsense_callback(self,msg):
        start_time = time.time()
        print("sim realsense callback",self.image_counter)
        cv_image_original = self.cv_bridge_.imgmsg_to_cv2(msg.img)
        depth_image_original = self.cv_bridge_.imgmsg_to_cv2(msg.depth)
        new_image = cv_image_original
        new_depth = depth_image_original
        if 'cx' not in self.cam_params:
            print("Not recieved intr yet, skipping frame")
            return
        if(new_depth.dtype == np.uint16):
            new_depth = new_depth / 1000.0
        h0, w0, _ = new_image.shape
        h1 = h0
        w1 = w0
        print("h1:", h1)
        print("w1:", w1)
        new_depth = cv2.resize(new_depth,(w1,h1))
        new_image = cv2.resize(new_image, (w1, h1))
        new_image = new_image[:h1-h1%8, :w1-w1%8]
        new_image = torch.as_tensor(new_image).permute(2, 0, 1)
        
        new_depth = torch.as_tensor(new_depth)
        new_depth = F.interpolate(new_depth[None,None], (h1, w1)).squeeze()
        new_depth = new_depth[:h1-h1%8, :w1-w1%8]

        intrinsics = torch.as_tensor([self.cam_params['fl_x'],self.cam_params['fl_y'],self.cam_params['cx'],self.cam_params['cy']])
        intrinsics[0::2] *= (w1 / w0)
        intrinsics[1::2] *= (h1 / h0)

        image = new_image[None]
        depth = new_depth

        start_time = time.time()
        #if not args.disable_vis:
        #    self.show_image(image[0])

        if self.t_ == 0:
            self.droid_args_.image_size = [image.shape[2], image.shape[3]]
            self.droid = Droid(self.droid_args_)
        
        self.droid.track(self.t_, image, depth, intrinsics=intrinsics)
        self.t_ += 1
        if(self.droid.video.counter.value == self.image_counter):
            if(self.last_pose is not None):
                ros_transform = TransformStamped()
                ros_transform.header.stamp = self.get_clock().now().to_msg()
                ros_transform.header.frame_id = "map_droid"
                ros_transform.child_frame_id = "droid_optical_frame"
                last_pose_arr = self.last_pose
                xyz = last_pose_arr[:3]
                quat = last_pose_arr[3:]
                matrix = np.eye(4)
                rotation = R.from_quat(quat)
                matrix[:3, :3] = rotation.as_matrix()
                matrix[:3, 3] = xyz
                map_to_droid_link = matrix
                map_to_base = np.linalg.inv(map_to_droid_link)# @ tilt_tf
                alt_position = map_to_base[:3,3]
                alt_rotation_matrix = map_to_base[:3,:3]
                alt_quaternion = R.from_matrix(alt_rotation_matrix).as_quat()

                ros_transform.transform.translation.x = alt_position[0]
                ros_transform.transform.translation.y = alt_position[1]
                ros_transform.transform.translation.z = alt_position[2]

                ros_transform.transform.rotation.x = alt_quaternion[0]
                ros_transform.transform.rotation.y = alt_quaternion[1]
                ros_transform.transform.rotation.z = alt_quaternion[2]
                ros_transform.transform.rotation.w = alt_quaternion[3]
        
                # matrix_quat = R.from_matrix(matrix[:3,:3]).as_quat()
                # matrix_pos = matrix[:3,3]
                # t.transform.translation.x = matrix_pos[0]
                # t.transform.translation.y = matrix_pos[1]
                # t.transform.translation.z = matrix_pos[2]

                # t.transform.rotation.x = matrix_quat[0]
                # t.transform.rotation.y = matrix_quat[1]
                # t.transform.rotation.z = matrix_quat[2]
                # t.transform.rotation.w = matrix_quat[3]

                # print('matrix pos', matrix_pos)
                self.tf_broadcaster.sendTransform(ros_transform)
                print("Sent transform",flush=True)
            return
        self.image_counter += 1
        pose = self.droid.video.poses[self.droid.video.counter.value-1].cpu().numpy()
        self.last_pose = pose
        print("Adding droid keyframe...")
        image_pose_msg = ImagePose()
        image_pose_msg.img = self.cv_bridge_.cv2_to_imgmsg(cv_image_original,encoding='bgr8')
        image_pose_msg.depth = self.cv_bridge_.cv2_to_imgmsg(depth_image_original)
        image_pose_msg.w = self.cam_params['w']
        image_pose_msg.h = self.cam_params['h']
        image_pose_msg.fl_x = self.cam_params['fl_x']
        image_pose_msg.fl_y = self.cam_params['fl_y']
        image_pose_msg.cx = self.cam_params['cx']
        image_pose_msg.cy = self.cam_params['cy']
        image_pose_msg.k1 = self.cam_params['k1']
        image_pose_msg.k2 = self.cam_params['k2']
        image_pose_msg.k3 = self.cam_params['k3']
        # import pdb; pdb.set_trace()
        posemat = self.xyzquat2mat(pose)
        pose = posemat[:3,3]
        print("xyz pose droidslam: ", pose)
        orient = R.from_matrix(posemat[:3,:3]).as_quat()
        if not (-0.001 < pose[0] < 0.001 and -0.001 < pose[1] < 0.001 and -0.001 < pose[2] < 0.001): # if pose is not [0,0,0] then publish
            image_pose_msg.pose = Pose(position=Point(x=pose[0],y=pose[1],z=pose[2]),orientation=Quaternion(x=orient[0],y=orient[1],z=orient[2],w=orient[3]))  # Replace with your Pose message
            self.publisher.publish(image_pose_msg)
        print("Total time: " + str(time.time() - start_time) +" seconds")

    def cam_intr_cb(self,msg):
        if 'w' in self.cam_params:
            return
        d_list = msg.d.tolist()
        if(len(d_list) > 0):
            k1 = msg.d[0]
            k2 = msg.d[1]
            k3 = msg.d[4]
        else:
            k1 = 0.0
            k2 = 0.0
            k3 = 0.0
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

    def image_callback_uncompressed(self, img_msg, depth_msg):
        realsense_sim_msg = ImagePose()
        # cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, desired_encoding='bgr8')  # Convert ROS Image message to OpenCV image
        # cv_img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        realsense_sim_msg.img = img_msg
        realsense_sim_msg.depth = depth_msg
        self.realsense_publisher.publish(realsense_sim_msg)
        return

    def doneCallback(self,msg):
        traj_est = self.droid.terminate(self.ros_image_stream(self.droid_args_.datapath, use_depth=False, stride=self.stride))
        print("#"*20 + " Results...")
    
        import evo
        from evo.core.trajectory import PoseTrajectory3D
        from evo.tools import file_interface
        from evo.core import sync
        import evo.main_ape as main_ape
        from evo.core.metrics import PoseRelation

        image_path = os.path.join(self.droid_args_.datapath, 'rgb')
        images_list = sorted(glob.glob(os.path.join(image_path, '*.png')))[::self.stride]
        tstamps = [float(x.split('/')[-1][:-4]) for x in images_list]

        traj_est = PoseTrajectory3D(
            positions_xyz=traj_est[:,:3],
            orientations_quat_wxyz=traj_est[:,3:],
            timestamps=np.array(tstamps))

        gt_file = os.path.join(self.droid_args_.datapath, 'groundtruth.txt')
        traj_ref = file_interface.read_tum_trajectory_file(gt_file)

        traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est)

        result = main_ape.ape(traj_ref, traj_est, est_name='traj', 
            pose_relation=PoseRelation.translation_part, align=True, correct_scale=False)

        print(result.stats)
        print("Did prime image stream")

    def ros_image_stream(self,datapath,use_depth=False,stride=1):
        fx, fy, cx, cy = np.loadtxt(os.path.join(datapath, 'calibration.txt')).tolist()
        image_list = sorted(glob.glob(os.path.join(datapath, 'rgb', '*.png')))[::stride]
        depth_list = sorted(glob.glob(os.path.join(datapath, 'depth', '*.png')))[::stride]

        for t, (image_file, depth_file) in enumerate(zip(image_list, depth_list)):
            
            image = cv2.imread(image_file)
            depth = cv2.imread(depth_file, cv2.IMREAD_ANYDEPTH) / 5000.0
            ros_image = self.cv_bridge_.cv2_to_imgmsg(image,encoding="passthrough")
            ros_image.header.frame_id = "droid_optical_frame"
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_depth = self.cv_bridge_.cv2_to_imgmsg(depth,encoding="passthrough")
            ros_depth.header.stamp = ros_image.header.stamp
            ros_depth.header.frame_id = "droid_optical_frame"
            ros_camera_info = CameraInfo()
            ros_camera_info.header.stamp = ros_image.header.stamp
            ros_camera_info.header.frame_id = "droid_optical_frame"
            ros_camera_info.k = [fx,0.0,cx,0.0,fy,cy,0.0,0.0,1.0]
            new_image = self.cv_bridge_.imgmsg_to_cv2(ros_image)
            new_depth = self.cv_bridge_.imgmsg_to_cv2(ros_depth)
            new_fx,_,new_cx,_,new_fy,new_cy,_,_,_ = ros_camera_info.k

            h0, w0, _ = new_image.shape
            h1 = h0
            w1 = w0
            print("h1:", h1)
            print("w1:", w1)
            new_image = cv2.resize(new_image, (w1, h1))
            new_image = new_image[:h1-h1%8, :w1-w1%8]
            new_image = torch.as_tensor(new_image).permute(2, 0, 1)
            
            new_depth = torch.as_tensor(new_depth)
            new_depth = F.interpolate(new_depth[None,None], (h1, w1)).squeeze()
            new_depth = new_depth[:h1-h1%8, :w1-w1%8]

            intrinsics = torch.as_tensor([new_fx, new_fy, new_cx, new_cy])
            intrinsics[0::2] *= (w1 / w0)
            intrinsics[1::2] *= (h1 / h0)
            if use_depth:
                yield t, new_image[None], new_depth, intrinsics

            else:
                yield t, new_image[None], intrinsics

    def fullImageCallback(self,ros_image,ros_depth,ros_camera_info):
        new_image = self.cv_bridge_.imgmsg_to_cv2(ros_image)
        new_depth = self.cv_bridge_.imgmsg_to_cv2(ros_depth)
        if(new_depth.dtype == np.uint16):
            new_depth = new_depth / 1000.0
        new_fx,_,new_cx,_,new_fy,new_cy,_,_,_ = ros_camera_info.k
        h0, w0, _ = new_image.shape
        h1 = h0
        w1 = w0
        print("h1:", h1)
        print("w1:", w1)
        new_depth = cv2.resize(new_depth,(w1,h1))
        new_image = cv2.resize(new_image, (w1, h1))
        new_image = new_image[:h1-h1%8, :w1-w1%8]
        new_image = torch.as_tensor(new_image).permute(2, 0, 1)
        
        new_depth = torch.as_tensor(new_depth)
        new_depth = F.interpolate(new_depth[None,None], (h1, w1)).squeeze()
        new_depth = new_depth[:h1-h1%8, :w1-w1%8]

        intrinsics = torch.as_tensor([new_fx, new_fy, new_cx, new_cy])
        intrinsics[0::2] *= (w1 / w0)
        intrinsics[1::2] *= (h1 / h0)

        image = new_image[None]
        depth = new_depth

        start_time = time.time()
        #if not args.disable_vis:
        #    self.show_image(image[0])

        if self.t_ == 0:
            self.droid_args_.image_size = [image.shape[2], image.shape[3]]
            self.droid = Droid(self.droid_args_)
        
        self.droid.track(self.t_, image, depth, intrinsics=intrinsics)
        self.t_ += 1
        if(self.droid.video.counter.value == self.image_counter):
            if(self.last_pose is not None):
                ros_transform = TransformStamped()
                ros_transform.header.stamp = self.get_clock().now().to_msg()
                ros_transform.header.frame_id = "map_droid"
                ros_transform.child_frame_id = "droid_optical_frame"
                last_pose_arr = self.last_pose
                xyz = last_pose_arr[:3]
                quat = last_pose_arr[3:]
                matrix = np.eye(4)
                rotation = R.from_quat(quat)
                matrix[:3, :3] = rotation.as_matrix()
                matrix[:3, 3] = xyz
                map_to_droid_link = matrix
                map_to_base = np.linalg.inv(map_to_droid_link)# @ tilt_tf
                alt_position = map_to_base[:3,3]
                alt_rotation_matrix = map_to_base[:3,:3]
                alt_quaternion = R.from_matrix(alt_rotation_matrix).as_quat()

                ros_transform.transform.translation.x = alt_position[0]
                ros_transform.transform.translation.y = alt_position[1]
                ros_transform.transform.translation.z = alt_position[2]

                ros_transform.transform.rotation.x = alt_quaternion[0]
                ros_transform.transform.rotation.y = alt_quaternion[1]
                ros_transform.transform.rotation.z = alt_quaternion[2]
                ros_transform.transform.rotation.w = alt_quaternion[3]
        
                # matrix_quat = R.from_matrix(matrix[:3,:3]).as_quat()
                # matrix_pos = matrix[:3,3]
                # t.transform.translation.x = matrix_pos[0]
                # t.transform.translation.y = matrix_pos[1]
                # t.transform.translation.z = matrix_pos[2]

                # t.transform.rotation.x = matrix_quat[0]
                # t.transform.rotation.y = matrix_quat[1]
                # t.transform.rotation.z = matrix_quat[2]
                # t.transform.rotation.w = matrix_quat[3]

                # print('matrix pos', matrix_pos)
                self.tf_broadcaster.sendTransform(ros_transform)
                print("Sent transform",flush=True)
            return
        self.image_counter += 1
        pose = self.droid.video.poses[self.droid.video.counter.value-1].cpu().numpy()
        self.last_pose = pose
        



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()