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
import open3d as o3d
import open3d.visualization as vis
import torch.nn.functional as F
import droid_backends
from lietorch import SE3

def create_point_actor(points, colors):
    """ open3d point cloud from numpy array """
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    point_cloud.colors = o3d.utility.Vector3dVector(colors)
    return point_cloud

class DroidNode(Node):
    def __init__(self, args):
        super().__init__('droid_node')
        self.droid = None
        self.args = args
        self.cam_transform = np.diag([1, -1, -1, 1])

        self.publisher = self.create_publisher(ImagePose, '/camera/color/imagepose', 10)
        self.pointcloud_publisher_ = self.create_publisher(PointCloud2, '/camera/pointcloud', 10)
        self.realsense_publisher = self.create_publisher(ImagePose, '/sim_realsense', 20)
        
        self.rgb_sub = message_filters.Subscriber(self, ROSImage, '/repub_image_raw')
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, '/ros2_camera/color/camera_info')
        self.depth_sub = message_filters.Subscriber(self,ROSImage,'/repub_depth_raw')
        self.cam_info_sub_2 = self.create_subscription(CameraInfo,'/ros2_camera/color/camera_info', self.cam_info_cb, 1)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.cam_info_sub], 20, 0.5)
        self.ts.registerCallback(self.per_timestep_callback)

        self.cam_params = {}
        self.bridge = CvBridge()
        self.t = 1

    def cam_info_cb(self, cam_info_msg):
        if 'w' in self.cam_params:
            return
        k1 = cam_info_msg.d[0]
        k2 = cam_info_msg.d[1]
        k3 = cam_info_msg.d[4]
        K = cam_info_msg.k.reshape(3,3)
        params = {
            "w": cam_info_msg.width,
            "h": cam_info_msg.height,
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

    def per_timestep_callback(self, rgb_msg, depth_msg, cam_info_msg):
        self.t += 1

        self.get_logger().info(f'received: {self.t}')
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

        K = cam_info_msg.k
        intrinsics = np.array([K[0], K[4], K[2], K[5]]) # unpack K matrix into [fx, fy, cx, cy]

        if self.droid is None:
            self.args.image_size = [rgb.shape[0], rgb.shape[1]]
            print('img size',self.args.image_size)
            self.droid = Droid(self.args)

        rgb_torch, depth_torch, intrinsics_torch = rgb.astype(np.uint8), depth.astype(np.float64), intrinsics.astype(np.float32)
        rgb_torch, depth_torch, intrinsics_torch = torch.as_tensor(rgb_torch).permute(2, 0, 1).unsqueeze(0), torch.as_tensor(depth_torch), torch.as_tensor(intrinsics_torch)

        # import pdb; pdb.set_trace()
        self.droid.track(self.t, rgb_torch, depth_torch, intrinsics=intrinsics_torch)

        pose = self.droid.video.poses[self.droid.video.counter.value-1].cpu().numpy()
        self.last_pose = pose
        if len(self.cam_params.keys()) > 0:
            image_pose_msg = ImagePose()
            image_pose_msg.img = self.bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
            image_pose_msg.depth = self.bridge.cv2_to_imgmsg(depth,encoding="16UC1")
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
            orient = R.from_matrix(posemat[:3,:3]).as_quat()
            image_pose_msg.pose = Pose(position=Point(x=pose[0],y=pose[1],z=pose[2]),orientation=Quaternion(x=orient[0],y=orient[1],z=orient[2],w=orient[3]))  # Replace with your Pose message
            self.last_disp = self.droid.video.disps[self.droid.video.counter.value-1].cpu().numpy()
            self.publisher.publish(image_pose_msg)
        print(self.t)
        if(self.t == 555):
            
            traj_est = self.droid.terminate(image_stream('/home/kushtimusprime/legs_ws/src/droid_slam_ros/datasets/ETH3D-SLAM/training/sfm_house_loop', use_depth=False, stride=1))
            import evo
            from evo.core.trajectory import PoseTrajectory3D
            from evo.tools import file_interface
            from evo.core import sync
            import evo.main_ape as main_ape
            from evo.core.metrics import PoseRelation
            image_path = '/home/kushtimusprime/leg_ws/src/droid_slam_ros/datasets/ETH3D-SLAM/training/sfm_house_loop/rgb'
            images_list = sorted(glob.glob(os.path.join(image_path, '*.png')))[::1]
            tstamps = [float(x.split('/')[-1][:-4]) for x in images_list]

    def xyzquat2mat(self,vec):
        xyz = vec[:3]
        quat = vec[3:]
        matrix = np.eye(4)
        try:
            rotation = R.from_quat(quat)
        except ValueError:
            rotation = R.from_quat([0, 0, 0, 1])
        # rotation = R.from_quat(quat[3] + quat[:4])
        matrix[:3, :3] = rotation.as_matrix()
        matrix[:3, 3] = xyz
        matrix = np.linalg.inv(matrix) @ self.cam_transform
        return matrix

    def saveJSON(self):
        print("In here")
        self.cam_params['frames']=self.frames
        with open(self.json_file_path_,"w") as json_file:
            json.dump(self.cam_params,json_file)
        print("Outta here")


def show_image(image):
    image = image.permute(1, 2, 0).cpu().numpy()
    cv2.imshow('image', image / 255.0)
    cv2.waitKey(1)

def image_stream(datapath, use_depth=False, stride=1):
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
        image = torch.as_tensor(image).permute(2, 0, 1)
        
        depth = torch.as_tensor(depth)
        depth = F.interpolate(depth[None,None], (h1, w1)).squeeze()
        depth = depth[:h1-h1%8, :w1-w1%8]

        intrinsics = torch.as_tensor([fx, fy, cx, cy])
        intrinsics[0::2] *= (w1 / w0)
        intrinsics[1::2] *= (h1 / h0)

        if use_depth:
            yield t, image[None], depth, intrinsics

        else:
            yield t, image[None], intrinsics

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
    import pdb
    pdb.set_trace()
    args.stereo = False

    torch.multiprocessing.set_start_method('spawn')

    #########################################################################
    rclpy.init()

    print('initializing node...')
    # Parse the arguments as before
    # ...
    import pdb; pdb.set_trace()
    node = DroidNode(args)
    try:
        rclpy.spin(node)  # Keep the node alive
    except KeyboardInterrupt:
        node.saveJSON()
        print("Exiting...")
    node.destroy_node()
    rclpy.shutdown()

    #########################################################################
    # datapath = './src/droid_slam_ros/datasets/ETH3D-SLAM/training/sfm_lab_room_1'
    # stride = 1
    # tstamps = []
    # for (t, image, depth, intrinsics) in tqdm(image_stream(datapath, use_depth=True, stride=stride)):
    #     if not args.disable_vis:
    #         show_image(image[0])

    #     if t == 0:
    #         args.image_size = [image.shape[2], image.shape[3]]
    #         droid = Droid(args)
        
    #     import pdb; pdb.set_trace()
    #     droid.track(t, image, depth, intrinsics=intrinsics)
    
    # traj_est = droid.terminate(image_stream(datapath, use_depth=False, stride=stride))

    # ### run evaluation ###

    # print("#"*20 + " Results...")

    # import evo
    # from evo.core.trajectory import PoseTrajectory3D
    # from evo.tools import file_interface
    # from evo.core import sync
    # import evo.main_ape as main_ape
    # from evo.core.metrics import PoseRelation

    # image_path = os.path.join(datapath, 'rgb')
    # images_list = sorted(glob.glob(os.path.join(image_path, '*.png')))[::stride]
    # tstamps = [float(x.split('/')[-1][:-4]) for x in images_list]

    # traj_est = PoseTrajectory3D(
    #     positions_xyz=traj_est[:,:3],
    #     orientations_quat_wxyz=traj_est[:,3:],
    #     timestamps=np.array(tstamps))

    # gt_file = os.path.join(datapath, 'groundtruth.txt')
    # traj_ref = file_interface.read_tum_trajectory_file(gt_file)

    # traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est)

    # result = main_ape.ape(traj_ref, traj_est, est_name='traj', 
    #     pose_relation=PoseRelation.translation_part, align=True, correct_scale=False)

    # print(result.stats)
    #########################################################################

if __name__ == '__main__':
    main()
