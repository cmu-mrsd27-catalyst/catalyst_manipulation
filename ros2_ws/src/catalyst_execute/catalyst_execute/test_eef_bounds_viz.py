#!/usr/bin/env python3
"""Visualize EEF bounds as a translucent marker box in RViz.

Publishes a MarkerArray on /eef_bounds_viz so you can see the
constraint region relative to the arm. Uses the hardcoded tag pose
and the same offsets as the action servers.

Usage:
  ros2 run catalyst_execute test_eef_bounds_viz

In RViz, add a MarkerArray display subscribing to /eef_bounds_viz.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


# Hardcoded tag pose (link_base; sync with gripper_pose / compute_poses)
TAG_X = -0.073853
TAG_Y = 0.475906
TAG_Z = 0.024722

# Offsets from tag (computed from boundary TCP poses)
X_MIN_OFFSET = -0.085
X_MAX_OFFSET = 0.135
Z_MIN_OFFSET = 0.141
Z_MAX_OFFSET = 0.381


class EefBoundsViz(Node):
    def __init__(self):
        super().__init__('eef_bounds_viz')

        self._pub = self.create_publisher(MarkerArray, '/eef_bounds_viz', 10)

        # Compute absolute bounds
        self._x_min = TAG_X + X_MIN_OFFSET
        self._x_max = TAG_X + X_MAX_OFFSET
        self._z_min = TAG_Z + Z_MIN_OFFSET
        self._z_max = TAG_Z + Z_MAX_OFFSET

        self.get_logger().info(
            f'EEF bounds: X[{self._x_min:.3f}, {self._x_max:.3f}] '
            f'Z[{self._z_min:.3f}, {self._z_max:.3f}]')

        # Publish at 2 Hz
        self.create_timer(0.5, self._publish)

    def _publish(self):
        ma = MarkerArray()

        # --- Translucent box showing the bounds volume ---
        box = Marker()
        box.header.frame_id = 'link_base'
        box.header.stamp = self.get_clock().now().to_msg()
        box.ns = 'eef_bounds'
        box.id = 0
        box.type = Marker.CUBE
        box.action = Marker.ADD

        # Box center
        box.pose.position.x = (self._x_min + self._x_max) / 2.0
        box.pose.position.y = TAG_Y  # center on tag Y
        box.pose.position.z = (self._z_min + self._z_max) / 2.0
        box.pose.orientation.w = 1.0

        # Box size
        box.scale.x = self._x_max - self._x_min
        box.scale.y = 0.5  # visible Y extent (bounds are unconstrained in Y)
        box.scale.z = self._z_max - self._z_min

        # Translucent green
        box.color.r = 0.0
        box.color.g = 1.0
        box.color.b = 0.0
        box.color.a = 0.15

        box.lifetime.sec = 1  # auto-expire if node stops
        ma.markers.append(box)

        # --- Wireframe edges using LINE_LIST ---
        edges = Marker()
        edges.header.frame_id = 'link_base'
        edges.header.stamp = box.header.stamp
        edges.ns = 'eef_bounds'
        edges.id = 1
        edges.type = Marker.LINE_LIST
        edges.action = Marker.ADD
        edges.scale.x = 0.002  # line width

        edges.color.r = 0.0
        edges.color.g = 0.8
        edges.color.b = 0.0
        edges.color.a = 0.8

        edges.lifetime.sec = 1

        # Y extent for visualization
        y_min = TAG_Y - 0.25
        y_max = TAG_Y + 0.25

        # 8 corners of the box
        corners = [
            (self._x_min, y_min, self._z_min),
            (self._x_max, y_min, self._z_min),
            (self._x_max, y_max, self._z_min),
            (self._x_min, y_max, self._z_min),
            (self._x_min, y_min, self._z_max),
            (self._x_max, y_min, self._z_max),
            (self._x_max, y_max, self._z_max),
            (self._x_min, y_max, self._z_max),
        ]

        # 12 edges of a box
        edge_indices = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # bottom
            (4, 5), (5, 6), (6, 7), (7, 4),  # top
            (0, 4), (1, 5), (2, 6), (3, 7),  # verticals
        ]

        from geometry_msgs.msg import Point
        for i, j in edge_indices:
            p1 = Point(x=corners[i][0], y=corners[i][1], z=corners[i][2])
            p2 = Point(x=corners[j][0], y=corners[j][1], z=corners[j][2])
            edges.points.append(p1)
            edges.points.append(p2)

        ma.markers.append(edges)

        # --- Tag position marker (small sphere) ---
        tag = Marker()
        tag.header.frame_id = 'link_base'
        tag.header.stamp = box.header.stamp
        tag.ns = 'eef_bounds'
        tag.id = 2
        tag.type = Marker.SPHERE
        tag.action = Marker.ADD
        tag.pose.position.x = TAG_X
        tag.pose.position.y = TAG_Y
        tag.pose.position.z = TAG_Z
        tag.pose.orientation.w = 1.0
        tag.scale.x = 0.02
        tag.scale.y = 0.02
        tag.scale.z = 0.02
        tag.color.r = 1.0
        tag.color.g = 0.0
        tag.color.b = 0.0
        tag.color.a = 1.0
        tag.lifetime.sec = 1
        ma.markers.append(tag)

        # --- Text labels for bounds ---
        for idx, (label, x, y, z) in enumerate([
            ('X_MIN', self._x_min, TAG_Y, (self._z_min + self._z_max) / 2),
            ('X_MAX', self._x_max, TAG_Y, (self._z_min + self._z_max) / 2),
            ('Z_MIN', (self._x_min + self._x_max) / 2, TAG_Y, self._z_min),
            ('Z_MAX', (self._x_min + self._x_max) / 2, TAG_Y, self._z_max),
        ]):
            txt = Marker()
            txt.header.frame_id = 'link_base'
            txt.header.stamp = box.header.stamp
            txt.ns = 'eef_bounds_labels'
            txt.id = idx
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = x
            txt.pose.position.y = y
            txt.pose.position.z = z
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.025  # text height
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = f'{label}: {x if "X" in label else z:.3f}'
            txt.lifetime.sec = 1
            ma.markers.append(txt)

        self._pub.publish(ma)


def main():
    rclpy.init()
    node = EefBoundsViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
