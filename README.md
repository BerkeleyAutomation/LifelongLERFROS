# Insta360 H264 Prime ROS Driver

Given an Insta360 ONE X2, this repo can be 
This repo can be built in a ROS2 humble workspace, so that when an Insta360 ONE X2 is connected with USB, the 2 fisheye images are published to separate ROS topics. The insta_fisheye node will publish the byte array containing the H264 streaming data. The h264_subscriber node will decode the byte array accordingly to generate an image that is then published. This has been tested on Ubuntu 22.04.

## Installation and Setup
It should just work with the default Ubuntu 22.04 setup, but I could be missing something.
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone <> src
```

## Running the code
```
colcon build
. install/setup.bash
ros2 launch insta_ros_driver full_launch.launch.py
```
If an Insta360 ONE X2 is plugged in with USB, it should start the livestream, and publish the front and back fisheye images to the /camera/image_raw1 and /camera/image_raw2 topics respectively.