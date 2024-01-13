#!/bin/bash

(trap 'kill 0' SIGINT; 

ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/usb_cam_left/image_raw/compressed --remap out:=/usb_cam_left/uncompressed &

ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/usb_cam_right/image_raw/compressed --remap out:=/usb_cam_right/uncompressed &

ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/imageo_compressedo --remap out:=/ros2_camera/color/uncompressed )
~                                                           
~                                            
