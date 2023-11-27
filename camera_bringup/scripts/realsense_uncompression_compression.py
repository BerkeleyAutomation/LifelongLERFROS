#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo

class CameraSync(Node):
    def __init__(self):
        super().__init__('camera_sync')

        # Subscribers
        self.image_sub = message_filters.Subscriber(self, Image, '/ros2_camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/ros2_camera/depth/image_rect_raw')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/ros2_camera/color/camera_info')

        # ApproximateTimeSynchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.info_sub], 10, 0.05, allow_headerless=True)
        self.ts.registerCallback(self.callback)

        # Publishers
        self.image_pub = self.create_publisher(Image, '/synced/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/synced/depth/image_rect_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/synced/color/camera_info', 10)
        self.i_ = 1

    def callback(self, image, depth, info):
        print(self.i_)
        # Republish with the same timestamp
        image.header.stamp = image.header.stamp
        depth.header.stamp = image.header.stamp
        info.header.stamp = image.header.stamp
        self.image_pub.publish(image)
        self.depth_pub.publish(depth)
        self.info_pub.publish(info)
        self.i_ += 1

def main(args=None):
    rclpy.init(args=args)
    camera_sync = CameraSync()
    rclpy.spin(camera_sync)
    camera_sync.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
