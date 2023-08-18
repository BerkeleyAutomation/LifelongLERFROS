# Lifelong LERF ROS
This repo will contain the ROS2 workspace code for the Lifelong LERF project. Currently, it can perform navigation on a Turtlebot4 Lite along with send images with a BRIO Webcam. This has been tested on Ubuntu 22.04.

## Turtlebot Installation and Setup
```
sudo apt-get install ros-humble-rplidar-ros #Should already be on there
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/BerkeleyAutomation/LifelongLERFROS.git src
```

To setup the Turtlebot to talk to the computer and vice versa, follow the instructions in this link: https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html.

## Computer Installation and Setup
```
sudo apt-get install ros-humble-turtlebot4-navigation
sudo apt-get install ros-humble-navigation2
sudo apt-get install ros-humble-nav2-bringup
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/BerkeleyAutomation/LifelongLERFROS.git src
```

## Run Navigation
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
