#!/usr/bin/env python3
"""Statistical outlier removal filter for PointCloud2 messages.

Subscribes to a raw point cloud, removes outlier points using a KNN-based
statistical method (same algorithm as PCL's StatisticalOutlierRemoval), and
publishes the filtered cloud for the octomap updater.

For each point, the mean distance to its K nearest neighbours is computed.
Points whose mean distance exceeds (global_mean + std_multiplier * global_std)
are rejected as outliers.

Defaults load from share/catalyst_vision/config/pointcloud_filter.yaml (or
source tree config/pointcloud_filter.yaml). ROS parameters still override.

Usage:
  ros2 run catalyst_vision pointcloud_filter

  ros2 run catalyst_vision pointcloud_filter --ros-args \
    -p input_topic:=/camera/camera/depth/color/points \
    -p output_topic:=/filtered_pointcloud \
    -p k_neighbours:=20 \
    -p std_multiplier:=1.0 \
    -p voxel_size:=0.005 \
    -p max_range:=0.8
"""

import os
import struct

import numpy as np
from scipy.spatial import cKDTree
import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def _load_pointcloud_filter_config():
    path = None
    try:
        share = get_package_share_directory('catalyst_vision')
        p = os.path.join(share, 'config', 'pointcloud_filter.yaml')
        if os.path.isfile(p):
            path = p
    except Exception:
        pass
    if path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        src = os.path.normpath(
            os.path.join(here, '..', 'config', 'pointcloud_filter.yaml'))
        if os.path.isfile(src):
            path = src

    defaults = {
        'input_topic': '/camera/camera/depth/color/points',
        'output_topic': '/filtered_pointcloud',
        'k_neighbours': 20,
        'std_multiplier': 1.0,
        'voxel_size': 0.005,
        'max_range': 0.8,
        'qos_depth': 5,
    }
    if not path:
        return defaults

    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    block = data.get('pointcloud_filter') or {}
    out = defaults.copy()
    for key in defaults:
        if key not in block:
            continue
        val = block[key]
        if key in ('k_neighbours', 'qos_depth'):
            out[key] = int(val)
        elif key in ('std_multiplier', 'voxel_size', 'max_range'):
            out[key] = float(val)
        else:
            out[key] = str(val)
    return out


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Extract XYZ points from a PointCloud2 message."""
    # Find xyz field offsets
    field_map = {f.name: f for f in msg.fields}
    if 'x' not in field_map or 'y' not in field_map or 'z' not in field_map:
        return np.empty((0, 3), dtype=np.float32)

    ox = field_map['x'].offset
    oy = field_map['y'].offset
    oz = field_map['z'].offset
    point_step = msg.point_step
    data = np.frombuffer(msg.data, dtype=np.uint8)

    n_points = msg.width * msg.height
    if n_points == 0:
        return np.empty((0, 3), dtype=np.float32)

    # Extract x, y, z as float32
    points = np.zeros((n_points, 3), dtype=np.float32)
    for i, offset in enumerate([ox, oy, oz]):
        # View the bytes at each field offset as float32
        indices = np.arange(n_points) * point_step + offset
        raw = np.array([data[idx:idx+4].tobytes() for idx in indices])
        points[:, i] = np.array([struct.unpack('f', b)[0] for b in raw])

    return points


def pointcloud2_to_xyz_fast(msg: PointCloud2) -> np.ndarray:
    """Fast XYZ extraction assuming standard float32 xyz layout."""
    field_map = {f.name: f for f in msg.fields}
    ox = field_map['x'].offset
    point_step = msg.point_step
    n_points = msg.width * msg.height
    if n_points == 0:
        return np.empty((0, 3), dtype=np.float32)

    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, point_step)
    # Extract 12 bytes (3 x float32) starting at x offset
    xyz_bytes = data[:, ox:ox+12].tobytes()
    points = np.frombuffer(xyz_bytes, dtype=np.float32).reshape(n_points, 3)
    return points.copy()


def xyz_to_pointcloud2(points: np.ndarray, header: Header) -> PointCloud2:
    """Create a PointCloud2 message from Nx3 float32 array."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.data = points.astype(np.float32).tobytes()
    msg.is_dense = True
    return msg


def voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Simple voxel grid downsampling."""
    if voxel_size <= 0 or len(points) == 0:
        return points
    # Quantize to voxel grid
    quantized = np.floor(points / voxel_size).astype(np.int32)
    # Unique voxels
    _, indices = np.unique(quantized, axis=0, return_index=True)
    return points[indices]


def statistical_outlier_removal(points: np.ndarray, k: int,
                                std_multiplier: float) -> np.ndarray:
    """Remove outliers using KNN mean distance thresholding.

    Same algorithm as PCL StatisticalOutlierRemoval:
    1. For each point, compute mean distance to K nearest neighbours
    2. Compute global mean and std of these mean distances
    3. Remove points where mean_dist > global_mean + std_multiplier * global_std
    """
    if len(points) < k + 1:
        return points

    tree = cKDTree(points)
    # k+1 because the first neighbour is the point itself
    dists, _ = tree.query(points, k=k + 1)
    # Skip self-distance (column 0)
    mean_dists = dists[:, 1:].mean(axis=1)

    global_mean = mean_dists.mean()
    global_std = mean_dists.std()
    threshold = global_mean + std_multiplier * global_std

    mask = mean_dists <= threshold
    return points[mask]


class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_filter')

        cfg = _load_pointcloud_filter_config()
        self.declare_parameter('input_topic', cfg['input_topic'])
        self.declare_parameter('output_topic', cfg['output_topic'])
        self.declare_parameter('k_neighbours', cfg['k_neighbours'])
        self.declare_parameter('std_multiplier', cfg['std_multiplier'])
        self.declare_parameter('voxel_size', cfg['voxel_size'])
        self.declare_parameter('max_range', cfg['max_range'])
        self.declare_parameter('qos_depth', cfg['qos_depth'])

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        qos_depth = self.get_parameter('qos_depth').value

        self._k = self.get_parameter('k_neighbours').value
        self._std_mul = self.get_parameter('std_multiplier').value
        self._voxel = self.get_parameter('voxel_size').value
        self._max_range = self.get_parameter('max_range').value

        self._pub = self.create_publisher(PointCloud2, output_topic, qos_depth)
        self.create_subscription(PointCloud2, input_topic, self._cb, qos_depth)

        self.get_logger().info(
            f'PointCloud filter: {input_topic} -> {output_topic}  '
            f'k={self._k}, std_mul={self._std_mul}, '
            f'voxel={self._voxel}m, max_range={self._max_range}m'
        )

    def _cb(self, msg: PointCloud2):
        points = pointcloud2_to_xyz_fast(msg)
        if len(points) == 0:
            return

        n_raw = len(points)

        # Remove NaN / inf
        valid = np.isfinite(points).all(axis=1)
        points = points[valid]

        # Range filter
        if self._max_range > 0:
            dists = np.linalg.norm(points, axis=1)
            points = points[dists <= self._max_range]

        # Voxel downsample (reduces KNN cost)
        if self._voxel > 0:
            points = voxel_downsample(points, self._voxel)

        # Statistical outlier removal
        points = statistical_outlier_removal(points, self._k, self._std_mul)

        # Publish
        out_msg = xyz_to_pointcloud2(points, msg.header)
        self._pub.publish(out_msg)

        self.get_logger().debug(
            f'Filter: {n_raw} -> {len(points)} points', throttle_duration_sec=5.0
        )


def main():
    rclpy.init()
    node = PointCloudFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
