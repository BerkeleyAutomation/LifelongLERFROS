#!/bin/bash

(trap 'kill 0' SIGINT; 
#ros2 run tf2_ros static_transform_publisher 0 0 0 -0.001, 0.210, 0.005, 0.978 base_footprint tilt_link &

# 4.5cm out from the tilt_link and 5.5cm above
# trans: [0.109, -0.004, 1.066]
# given this RPY (degree): [-0.000, 24.602, -1.558]
# overall diff is [0.154, -0.004, 1.121]
# ros2 run tf2_ros static_transform_publisher 0 0 0 0.5 -0.5 -0.5 0.5 base_footprint ros2_camera_link &
ros2 run tf2_ros static_transform_publisher 0.154 0 1.121 0.001 0.174 -0.008 0.985 base_footprint ros2_camera_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 -0.5 0.5 -0.5 0.5 ros2_camera_link ros2_pointcloud &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map odom &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom base_footprint 
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom droid_slam_link
) 
