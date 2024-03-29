# Lifelong LERF ROS
This repo will contain the ROS2 workspace code for the Lifelong LERF project. Currently, it can stream color and depth images from a Realsense D457 and teleop move a Turtlebot4 with keyboard commands. This has been tested on Ubuntu 22.04.

Note: Occasionally this repo uses the convention `sr1` and `sr2`. This is equivalent to `source /opt/ros/noetic/setup.bash` and `source ~/ros2_foxy/install/setup.bash && source /opt/ros/foxy/setup.bash` respectively.

## Turtlebot Installation and Setup
To setup the Turtlebot to talk to the computer and vice versa, follow the instructions in this link: https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html.
If Donatello is nearby, follow these instructions to setup and install necessary libraries
```
ssh ubuntu@10.65.87.91
sudo apt-get install ros-humble-realsense2-camera
sudo apt-get install ros-humble-librealsense2* #Should already be on there
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/BerkeleyAutomation/LifelongLERFROS.git src
```

## Computer Installation and Setup
```
sudo apt-get install ros-humble-turtlebot4-navigation
sudo apt-get install ros-humble-navigation2
sudo apt-get install ros-humble-nav2-bringup
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/BerkeleyAutomation/LifelongLERFROS.git src
cd src
source env_setup.bash
```

## Run Camera (Realsense D457/D435)
ON TURTLEBOT
```
ssh ubuntu@10.65.87.91
cd ~/ros2_ws
colcon build
. install/setup.bash
ros2 launch robot_bringup standard_realsense.launch.py
```

ON COMPUTER
```
cd ~/ros2_ws
colcon build
. install/setup.bash
ros2 launch camera_bringup image_visualization.launch.py
```
A color and depth image window will open showing the camera streams.

## Install DROID-SLAM
Mamba setup from this link: https://robofoundry.medium.com/using-robostack-for-ros2-9bb52ca89c12

curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh"

bash Mambaforge-$(uname)-$(uname -m).sh

source ~/.bashrc

conda install mamba -c conda-forge

mamba create -n droid_slam_ros_env python=3.10.12

mamba activate droid_slam_ros_env

python -m pip install --upgrade pip


Nerfstudio Setup from this link: https://docs.nerf.studio/quickstart/installation.html

pip uninstall torch torchvision functorch tinycudann

pip install torch==2.0.1+cu118 torchvision==0.15.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118

conda install -c "nvidia/label/cuda-11.8.0" cuda-toolkit

pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch

pip install nerfstudio


Mix of this link: https://robofoundry.medium.com/using-robostack-for-ros2-9bb52ca89c12

And then this link: https://robostack.github.io/GettingStarted.html

conda config --env --add channels conda-forge

conda config --env --add channels robostack

conda config --env --add channels robostack-humble

conda config --env --add channels robostack-experimental

mamba install ros-humble-desktop-full # Will fail

conda config --env --add channels conda-forge

conda config --env --add channels robostack-staging

conda config --env --remove channels defaults

mamba install ros-humble-desktop

mamba install ros-humble-desktop-full


Pip install the rest of the stuff

pip install torch-scatter==2.1.1

pip install matplotlib==3.7.2

pip install matplotlib-inline==0.1.6

pip install lietorch==0.6.2

## Run Droid-SLAM
ON TURTLEBOT (Needs camera images as input)
```
ros2 launch robot_bringup standard_realsense.launch.py
```

ON COMPUTER (Terminal 1)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
ON COMPUTER (Terminal 2)
```
ros2 launch droid_slam_ros droid_slam.launch.py
```
From there, you should get a Viser link and can view Droid-SLAM in action.
## Installation

## Setting up Joystick
Connect to the robot and run these commands to be able to teleop the fetch.
```
ssh fetch@fetch59.local # password robotics
sr1
sudo systemctl restart roscore.service
sudo systemctl restart robot.service
```
Then press the center playstation button on the joystick and if you see a lone solid red light then you're connected.
Note: if this doesn't work then just `sudo reboot` the fetch and it should work first try.

## Teleop with Keyboard
Teleop'ing with keyboard results in far smoother movements of the fetch than using the joystick so it is recommended you move the robot with this when possible. ssh into the fetch, `sr1` and run
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
and you should be all set!

## Setting Up Gstreamer
We need to Gstream the main arducam for high FPS to do DROID-SLAM. The other 3 cameras can run slower and only be used for the LEGS. 
To send images, on fetch run (ensure that the specified device is the front-facing arducam): 
```
sudo gst-launch-1.0 v4l2src device=/dev/video0 ! image/jpeg,width=640,height=480,framerate=15/1 ! jpegdec ! video/x-raw ! videoconvert ! x264enc tune=zerolatency bitrate=400000 ! rtph264pay config-interval=1 ! udpsink host=10.65.87.27 port=5001 sync=false
```

To receive images, on the computer run: 
```
gst-launch-1.0 udpsrc port=5001 ! \
    application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! \
    videoconvert ! autovideosink
```

## Setting Up Arducams:

For 4 Arducam Setup: 
There is a specific order that all of the cameras need to be plugged into. The back camera needs to be plugged into Justin's usb-c dongle at in the closest port to the usb-c connector. The dongle is then plugged into the usb-c port labeled 2 (one closer to the center of the robot). Front camera plugs into the bottom usb-a connector on the leftside of the nuc (left from robot frame). Left camera plugs into the port right above the left camera and the right camera plugs into the only free port on the right. 

Connect to the robot and remap the ports.  
```
ssh fetch@fetch59.local # password robotics
sr2
cd ~/ros2_ws
colcon_build
. install/setup.bash
cd src/camera_bringup/scripts
sudo bash remap_cameras_4_arducam.bash
```
After that, while still in the fetch run the launch script
```
cd ~/ros2_ws
colcon_build
. install/setup.bash
source /opt/ros/foxy/setup.bash
ros2 launch camera_bringup 4_camera_launch.py
```
If you get the error: `terminate unrecognized character "char*"`. Run this command:
```
sudo usermod -a -G video $LOGNAME
```
To confirm this works run `groups` you should see `video` listed. If not then log back out and log back in and rerun the commands for the launch file. 

On the computer then go into the lifelong_lerf_ws and run the script to uncompress the images
```
cd ~/lifelong_lerf_ws
colcon build
. install/setup.bash
ros2 run camera_bringup 4_arducam_compressed_converter.py
```

You should now see all four cameras publishing on `/repub/cam<direction>/image_raw`.

For 3 Arducam Setup:
This is when we're using the realsense as our front camera. In this case have the realsense plugged into the left top usb port with the left camera plugged right below. Have the back camera plugged into the close end of the usb-c splitter and the right camera plugged into the other spot. Have the splitter plugged into usb-c port 2.

Connect to the robot and remap the ports.  
```
ssh fetch@fetch59.local # password robotics
sr2
cd ~/ros2_ws
colcon_build
. install/setup.bash
cd src/camera_bringup/scripts
sudo bash remap_cameras_3_arducam.bash
```
After that, while still in the fetch run the launch script
```
cd ~/ros2_ws
colcon_build
. install/setup.bash
source /opt/ros/foxy/setup.bash
ros2 launch camera_bringup 3_camera_launch.py
```
On the computer then go into the lifelong_lerf_ws and run the script to uncompress the images
```
cd ~/lifelong_lerf_ws
colcon build
. install/setup.bash
ros2 run camera_bringup 3_arducam_compressed_converter.py
```

You can see their videos by running `view_4_cam.py` located in `camera_bring/scripts/` (for some reason ros doesn't like working with this)


## Rtabmap Installation
```
sudo apt-get install ros-humble-rtabmap-ros
```

## Run Navigation 11/18 (Still under development)

Okay, so we got a lot of moving pieces to get this working. We will consolidate soon.
First, make sure the Realsense is plugged into the Fetch and run it.
```
ssh fetch@fetch59.local # password robotics
source ~/ros2_foxy/install/setup.bash
cd ~/ros2_ws
colcon build
. install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```
Note: to specify the resolution of the color/depth of the camera you can run `ros2 launch realsense2_camera rs_launch.py depth_module.profile:=848x480x30 rgb_camera.profile:=848x480x30` 
To verify this works, in another window, ssh into the fetch, source ros2_foxy, and check the frequency of /ros2_camera/color/image_raw. It should be at about 15 Hz.

Next, run the image compression node on the Fetch. We need this because directly subscribing to the full image on the computer causes too much lag, so we subscribe to the compressed image on the computer.
```
ssh fetch@fetch59.local # password robotics
source ~/ros2_foxy/install/setup.bash
cd ~/ros2_ws
colcon build
. install/setup.bash
ros2 run image_compression color_image_compression_node.py
```

To verify this works, in another window, ssh into the fetch, source ros2_foxy, and check the frequency of /imageo_compressedo. It should also be at about 15 Hz.

Next, ssh into the fetch and run the ros2 to ros1 bridge.
```
ssh fetch@fetch59.local
source /opt/ros/noetic/setup.bash
rosparam load ~/ros2_ws/src/image_compression/params/bridge.yaml
source ~/ros2_foxy/install/setup.bash
ros2 run ros1_bridge parameter_bridge
```

Then, run the talker node in ROS2 on the computer and the listener on ROS1 on the Fetch.
```
ros2 run demo_nodes_cpp talker
```

```
ssh fetch@fetch59.local
source /opt/ros/noetic/setup.bash
rosrun roscpp_tutorials listener
```
You should be seeing messages on both the ROS1 and ROS2 ends indicating the bridge is working.

Next, on the computer, run the image uncompression node.
```
cd ~/lifelong_lerf_ws
colcon build
. install/setup.bash
ros2 run camera_bringup realsense_compressed_converter.py
```

To verify this works, in another window, check the frequency of /repub_image_raw. It should also be at about 15 Hz.

Next, on the computer in another window, run RTABMAP and then run rviz and make sure you can visualize the map.
```
cd ~/lifelong_lerf_ws
colcon build
. install/setup.bash
ros2 launch realsense_rtabmap_slam_bringup new_rtabmap.launch.py
```

Next, on computer in another window, run navigation. You should see the window say the words "Creating bond timer..."
```
cd ~/lifelong_lerf_ws
colcon build
. install/setup.bash
ros2 launch realsense_rtabmap_slam_bringup navigation.launch.py
```

To verify navigation is working, echo the following topics on the computer: /cmd_vel and /navigate_to_pose/_action/status

Now we need to verify that the bridge can still work, so kill the chatter topic talker, and then rerun it, and make sure the listener still works.

Now that you have verified this, you can permanently kill the talker.

In another window, run the twist to string conversion on the computer.
```
cd ~/lifelong_lerf_ws
colcon build
. install/setup.bash
ros2 run realsense_rtabmap_slam_bringup twist_to_string.py
```

Then, on the fetch, run the corresponding string to twist conversion.
```
ssh fetch@fetch59.local
source /opt/ros/noetic/setup.bash
cd lifelong_lerf_fetch_ws
catkin_make
source devel/setup.bash
rosrun nuc_bridge string_to_twist.py
```

Now, you should put a goal down in RVIZ, and it should navigate to the goal!!! You can verify that you reached the goal when the /navigate_to_pose/_action/status has a status 4 as opposed to staus 2. Status 6 means that the goal was aborted


sudo apt-get install ros-humble-octomap-mapping

ros2 run tf2_ros static_transform_publisher 0 0 0 0.5 -0.5 -0.5 0.5 base_footprint ros2_camera_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0.5 -0.5 0.5 -0.5 ros2_camera_link ros2_pointcloud


## RTABMap + DROID-SLAM (1/22)
`ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom`
`ros2 run tf2_ros static_transform_publisher 0 0 1.1557 -1.57 0 -1.57 odom map_droid`
`ros2 run tf2_ros static_transfoheransform_publisher 0 0 0 -1.57 0 -1.57 map map_droid`
=== on desktop ===
in each new terminal: conda deactivate; cd ~/ros2_ws (on duchamp1 this is called legs_ws); . install/setup.bash

then:
mamba activate droid_slam_ros_env
cd droid_slam_ros
python setup.py install
ros2 run droid_slam_ros droid_slam_node.py

in another terminal:
mamba activate droid_slam_ros_env
ros2 run image_transport_tutorials depth_decode_node 

