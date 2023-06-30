#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

class mynode(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.pose_subscriber_ = self.create_subscription(
                PointCloud2,
                "/velodyne_points",
                self.callback,
                10)
    def callback(self, msg: PointCloud2):
        self.get_logger().info(str(msg))


def main(args=None):
    rclpy.init(args=args)
    node = mynode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
