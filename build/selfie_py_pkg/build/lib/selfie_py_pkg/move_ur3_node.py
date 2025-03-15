#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)
    node = Node("node_name")
    node.get_logger().info("Hellow wolrd")
    rclpy.shutdown()

if __name__ == "__main__":
    main()