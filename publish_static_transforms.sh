#!/bin/bash

(trap 'kill 0' SIGINT; 
#ros2 run tf2_ros static_transform_publisher 0 0 0 -0.001, 0.210, 0.005, 0.978 base_footprint tilt_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0.5 -0.5 -0.5 0.5 base_footprint ros2_camera_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 -0.5 0.5 -0.5 0.5 base_footprint ros2_pointcloud &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map odom ) 
