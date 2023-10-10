# Lifelong LERF ROS
This repo will contain the ROS2 workspace code for the Lifelong LERF project. Currently, it can stream color and depth images from a Realsense D457 and teleop move a Turtlebot4 with keyboard commands. This has been tested on Ubuntu 22.04.

## Turtlebot Installation and Setup
To setup the Turtlebot to talk to the computer and vice versa, follow the instructions in this link: https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html.
If Donatello is nearby, follow these instructions to setup and install necessary libraries
```
ssh ubuntu@10.65.87.91
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

## Run Navigation (Still under development)
ON TURTLEBOT
```
cd ~/ros2_ws
colcon build
. install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.xml
```

ON COMPUTER
```
cd ~/ros2_ws
colcon build
. install/setup.bash
ros2 launch navigation_bringup navigation_bringup.launch.xml
```
After launching navigation bringup, it will take like 30 seconds for it to get setup so you can place a goal. Once the terminal says it is resizing the costmap, you are good to go. To place a goal in RVIZ, click the Navigation2 Goal button and place the arrow on the map. Once set, the robot should start moving. To monitor robot progress, echo the `navigate_to_pose/_action/status` topic. Status 2 means it is in progress, Status 6 means it aborted to goal, and Status 4 means Goal Reached.

Additional Note: The default turtlebot service may cause issues with the ability to generate consistent LiDAR scan. You can turn off this service with `sudo systemctl stop turtlebot4.service`

