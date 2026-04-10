#!/usr/bin/env python3
"""Capture RGB and aligned depth images from the RealSense camera.

Press Enter to capture a pair of images. Press 'q' + Enter to quit.
Images are saved under save_dir from config (default ~/test_images_catalyst) as:
  color_000.png, depth_000.png, color_001.png, depth_001.png, ...

Usage:
  ros2 run catalyst_vision capture_images
"""

import os

import cv2
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def _load_capture_config():
    path = None
    try:
        share = get_package_share_directory('catalyst_vision')
        p = os.path.join(share, 'config', 'capture_images.yaml')
        if os.path.isfile(p):
            path = p
    except Exception:
        pass
    if path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        src = os.path.normpath(
            os.path.join(here, '..', 'config', 'capture_images.yaml'))
        if os.path.isfile(src):
            path = src
    defaults = {
        'save_dir': '~/test_images_catalyst',
        'color_topic': '/camera/camera/color/image_raw',
        'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
        'image_qos_depth': 5,
    }
    if not path:
        return defaults
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    block = data.get('capture_images') or {}
    out = defaults.copy()
    for k in defaults:
        if k in block:
            out[k] = block[k]
    return out


class ImageCapture(Node):
    def __init__(self):
        super().__init__('image_capture')
        cfg = _load_capture_config()
        self._save_dir = os.path.expanduser(str(cfg['save_dir']))
        self._color_topic = str(cfg['color_topic'])
        self._depth_topic = str(cfg['depth_topic'])
        qos_depth = int(cfg['image_qos_depth'])

        self._bridge = CvBridge()
        self._color_img = None
        self._depth_img = None

        self.create_subscription(
            Image, self._color_topic, self._color_cb, qos_depth)
        self.create_subscription(
            Image, self._depth_topic, self._depth_cb, qos_depth)

        os.makedirs(self._save_dir, exist_ok=True)
        self.get_logger().info(f'Saving images to {self._save_dir}')
        self.get_logger().info(
            f'Subscribed to {self._color_topic} and {self._depth_topic}')
        self.get_logger().info('Press Enter to capture, q + Enter to quit.')

    def _color_cb(self, msg: Image):
        self._color_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg: Image):
        self._depth_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


def main():
    rclpy.init()
    node = ImageCapture()

    count = 0
    save_dir = node._save_dir
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
            color_path = os.path.join(save_dir, f'color_{count:03d}.png')
            cv2.imwrite(color_path, node._color_img)

            # Save depth as 16-bit PNG (millimetres)
            depth_path = os.path.join(save_dir, f'depth_{count:03d}.png')
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
