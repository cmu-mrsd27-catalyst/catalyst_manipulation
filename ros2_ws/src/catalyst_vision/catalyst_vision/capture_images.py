#!/usr/bin/env python3
"""Capture RGB and aligned depth images from the RealSense camera.

Press Enter to capture a pair of images. Press 'q' + Enter to quit.
Images are saved to ~/test_images_catalyst/ as:
  color_000.png, depth_000.png, color_001.png, depth_001.png, ...

Usage:
  ros2 run catalyst_vision capture_images
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

SAVE_DIR = os.path.expanduser('~/test_images_catalyst')

COLOR_TOPIC = '/camera/camera/color/image_raw'
DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'


class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture')
        self._bridge = CvBridge()
        self._color_img = None
        self._depth_img = None

        self.create_subscription(Image, COLOR_TOPIC, self._color_cb, 5)
        self.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, 5)

        os.makedirs(SAVE_DIR, exist_ok=True)
        self.get_logger().info(f'Saving images to {SAVE_DIR}')
        self.get_logger().info(f'Subscribed to {COLOR_TOPIC} and {DEPTH_TOPIC}')
        self.get_logger().info('Press Enter to capture, q + Enter to quit.')

    def _color_cb(self, msg: Image):
        self._color_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg: Image):
        self._depth_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


def main():
    rclpy.init()
    node = ImageCapture()

    count = 0
    try:
        while True:
            # Spin to get latest images
            rclpy.spin_once(node, timeout_sec=0.1)

            user_input = input()
            if user_input.strip().lower() == 'q':
                break

            # Spin a few more times to ensure we have the latest frame
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.05)

            if node._color_img is None:
                node.get_logger().warn('No color image received yet')
                continue
            if node._depth_img is None:
                node.get_logger().warn('No depth image received yet')
                continue

            # Save color as RGB PNG
            color_path = os.path.join(SAVE_DIR, f'color_{count:03d}.png')
            cv2.imwrite(color_path, node._color_img)

            # Save depth as 16-bit PNG (millimetres)
            depth_path = os.path.join(SAVE_DIR, f'depth_{count:03d}.png')
            cv2.imwrite(depth_path, node._depth_img)

            node.get_logger().info(
                f'[{count}] Saved {color_path} and {depth_path}'
            )
            count += 1

    except (KeyboardInterrupt, EOFError):
        pass

    node.get_logger().info(f'Captured {count} image pairs.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
