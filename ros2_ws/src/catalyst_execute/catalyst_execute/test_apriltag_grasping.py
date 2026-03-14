#!/usr/bin/env python3
"""
Test script for AprilTag-based grasping.

Subscribes to /world_model to get AprilTag poses in link_base frame.
Uses pre-defined grasp transforms (H_TCP_TO, H_TCP_TB) from gripper_pose.json
to compute target TCP poses relative to detected tags.

If both tags (tag_object, tag_base) are visible, computes the grasp pose
from each and averages them. If only one is visible, uses that one alone.
"""

import json
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node

import tf2_ros
from rclpy.time import Time

from std_msgs.msg import String
from catalyst_interfaces.srv import JsonCommand

JOINT_SERVICE = '/joint_command'
CARTESIAN_SERVICE = '/cartesian_command'
GRIPPER_SERVICE = '/gripper_command'
GUIDE_MODE_SERVICE = '/guide_mode'


def quat_to_rotation_matrix(qx, qy, qz, qw):
    """Convert quaternion to 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


def rotation_matrix_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion (qx, qy, qz, qw)."""
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def pose_to_homogeneous(position, orientation):
    """Convert position/orientation dicts to 4x4 homogeneous matrix."""
    H = np.eye(4)
    H[:3, :3] = quat_to_rotation_matrix(
        orientation['qx'], orientation['qy'],
        orientation['qz'], orientation['qw']
    )
    H[0, 3] = position['x']
    H[1, 3] = position['y']
    H[2, 3] = position['z']
    return H


def homogeneous_to_pos_quat(H):
    """Extract (x, y, z, qx, qy, qz, qw) from a 4x4 homogeneous matrix."""
    pos = H[:3, 3]
    qx, qy, qz, qw = rotation_matrix_to_quat(H[:3, :3])
    return pos[0], pos[1], pos[2], qx, qy, qz, qw


def average_quaternions(q1, q2):
    """Average two quaternions. Ensures they're in the same hemisphere before averaging."""
    q1 = np.array(q1)
    q2 = np.array(q2)
    # Flip q2 if dot product is negative (opposite hemisphere)
    if np.dot(q1, q2) < 0:
        q2 = -q2
    avg = (q1 + q2) / 2.0
    return avg / np.linalg.norm(avg)


def average_poses(H1, H2):
    """Average two homogeneous transforms: mean position, averaged quaternion."""
    pos1 = H1[:3, 3]
    pos2 = H2[:3, 3]
    avg_pos = (pos1 + pos2) / 2.0

    q1 = rotation_matrix_to_quat(H1[:3, :3])
    q2 = rotation_matrix_to_quat(H2[:3, :3])
    avg_q = average_quaternions(q1, q2)

    H = np.eye(4)
    H[:3, :3] = quat_to_rotation_matrix(avg_q[0], avg_q[1], avg_q[2], avg_q[3])
    H[:3, 3] = avg_pos
    return H


def compute_look_at_camera_pose(tag_position, tag_rotation, standoff=0.25, height_offset=0.05):
    """Compute a desired camera_color_optical_frame pose that looks at the tag head-on.

    Positions the camera horizontally in front of the tags (no tilt), backed off
    along the horizontal projection of the tag normal. The camera stays level at
    the same height as the tags (plus height_offset).

    The returned frame follows optical convention: Z forward, X right, Y down.

    Args:
        tag_position: (3,) array — tag center in link_base frame
        tag_rotation: (3,3) array — tag rotation in link_base frame
        standoff: horizontal distance from the tags to place the camera
        height_offset: extra upward (world-Z) shift for the viewpoint

    Returns:
        4x4 homogeneous matrix — desired camera_color_optical_frame pose in link_base
    """
    tag_z = tag_rotation[:, 2]  # tag normal

    # Project tag normal onto XY plane to get horizontal back-off direction
    horizontal_dir = tag_z.copy()
    horizontal_dir[2] = 0.0
    norm = np.linalg.norm(horizontal_dir)
    if norm < 1e-6:
        # Tag normal is purely vertical; fall back to backing off along base-X
        horizontal_dir = np.array([1.0, 0.0, 0.0])
    else:
        horizontal_dir = horizontal_dir / norm

    # Camera position: back off horizontally, stay at tag height + offset
    cam_pos = tag_position + standoff * horizontal_dir
    cam_pos[2] = tag_position[2] + height_offset

    # Camera Z axis: points horizontally from camera toward tag center (no tilt)
    cam_z = tag_position - cam_pos
    cam_z[2] = 0.0  # force level — no vertical component
    cam_z = cam_z / np.linalg.norm(cam_z)

    # Camera Y axis: points down (optical convention)
    cam_y = np.array([0.0, 0.0, -1.0])

    # Camera X axis: completes right-handed frame (points right)
    cam_x = np.cross(cam_y, cam_z)
    cam_x = cam_x / np.linalg.norm(cam_x)

    H = np.eye(4)
    H[:3, 0] = cam_x
    H[:3, 1] = cam_y
    H[:3, 2] = cam_z
    H[:3, 3] = cam_pos
    return H


class AprilTagGraspNode(Node):
    def __init__(self):
        super().__init__('test_apriltag_grasping')

        # Load grasp transforms from JSON
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'gripper_pose.json'
        )
        if not os.path.exists(config_path):
            from ament_index_python.packages import get_package_share_directory
            config_path = os.path.join(
                get_package_share_directory('catalyst_execute'),
                'config', 'gripper_pose.json'
            )

        with open(config_path, 'r') as f:
            grasp_data = json.load(f)

        self._H_TCP_TO = np.array(grasp_data['H_TCP_TO'])  # TCP w.r.t. tag_object
        # self._H_TCP_TB = np.array(grasp_data['H_TCP_TB'])  # TCP w.r.t. tag_base
        self.get_logger().info(f'Loaded grasp transforms from {config_path}')

        # TF2 buffer for looking up transforms between frames
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # World model state
        self._world_model = None
        self.create_subscription(String, '/world_model', self._world_model_cb, 10)

        # Service clients
        self._joint_client = self.create_client(JsonCommand, JOINT_SERVICE)
        self._cartesian_client = self.create_client(JsonCommand, CARTESIAN_SERVICE)
        self._gripper_client = self.create_client(JsonCommand, GRIPPER_SERVICE)
        self._guide_mode_client = self.create_client(JsonCommand, GUIDE_MODE_SERVICE)

    def _world_model_cb(self, msg: String):
        self._world_model = json.loads(msg.data)

    def wait_for_services(self):
        self.get_logger().info('Waiting for motion planner services...')
        self._joint_client.wait_for_service()
        self._cartesian_client.wait_for_service()
        self._gripper_client.wait_for_service()
        self._guide_mode_client.wait_for_service()
        self.get_logger().info('All services ready.')

    def wait_for_world_model(self, timeout=10.0):
        self.get_logger().info('Waiting for world model data...')
        start = time.time()
        while self._world_model is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('Timed out waiting for /world_model')
                return False
        self.get_logger().info('World model data received.')
        return True

    def _get_tag_H(self, tag_frame):
        """Get tag's 4x4 homogeneous matrix in link_base from world model, or None."""
        rclpy.spin_once(self, timeout_sec=0.5)
        if self._world_model is None:
            return None
        for tag in self._world_model.get('apriltags', []):
            if tag['frame'] == tag_frame and tag['pose_in_base'] is not None:
                pose = tag['pose_in_base']
                return pose_to_homogeneous(pose['position'], pose['orientation'])
        return None

    def compute_grasp_pose(self, num_samples=15, sample_interval=0.1):
        """Compute target TCP pose in link_base using visible AprilTags.

        Takes multiple samples, rejects position outliers (beyond 1.5 * IQR),
        and averages the remaining poses for a stable estimate.

        Args:
            num_samples: number of detection readings to collect
            sample_interval: seconds between readings

        Returns:
            (H_grasp, H_to, H_tb) — averaged grasp pose and last raw tag poses
        """
        self.get_logger().info(f'Collecting {num_samples} tag samples...')

        samples_to = []  # list of 4x4 grasp poses from tag_object
        samples_tb = []  # list of 4x4 grasp poses from tag_base
        last_H_to = None
        last_H_tb = None

        for i in range(num_samples):
            H_to = self._get_tag_H('tag_object')
            H_tb = self._get_tag_H('tag_base')

            if H_to is not None:
                last_H_to = H_to
                samples_to.append(H_to @ self._H_TCP_TO)
            if H_tb is not None:
                last_H_tb = H_tb
                samples_tb.append(H_tb @ self._H_TCP_TB)

            time.sleep(sample_interval)

        self.get_logger().info(
            f'Collected {len(samples_to)} tag_object, {len(samples_tb)} tag_base samples'
        )

        # Merge all grasp pose samples
        all_samples = samples_to + samples_tb
        if not all_samples:
            self.get_logger().error('No AprilTags visible in any sample')
            return None, last_H_to, last_H_tb

        # Extract positions and filter outliers using IQR on Euclidean distance from median
        positions = np.array([H[:3, 3] for H in all_samples])
        median_pos = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - median_pos, axis=1)

        q1 = np.percentile(distances, 25)
        q3 = np.percentile(distances, 75)
        iqr = q3 - q1
        threshold = q3 + 1.5 * iqr

        inlier_mask = distances <= threshold
        inliers = [H for H, keep in zip(all_samples, inlier_mask) if keep]
        n_rejected = len(all_samples) - len(inliers)

        if n_rejected > 0:
            self.get_logger().info(f'Rejected {n_rejected} outlier(s) out of {len(all_samples)} samples')

        if not inliers:
            self.get_logger().warn('All samples were outliers — using all samples anyway')
            inliers = all_samples

        # Average the inlier poses: mean position, iteratively averaged quaternion
        avg_pos = np.mean([H[:3, 3] for H in inliers], axis=0)
        avg_q = np.array(rotation_matrix_to_quat(inliers[0][:3, :3]))
        for H in inliers[1:]:
            avg_q = average_quaternions(avg_q, rotation_matrix_to_quat(H[:3, :3]))

        H_grasp = np.eye(4)
        H_grasp[:3, :3] = quat_to_rotation_matrix(avg_q[0], avg_q[1], avg_q[2], avg_q[3])
        H_grasp[:3, 3] = avg_pos

        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_grasp)
        self.get_logger().info(
            f'Averaged grasp pose ({len(inliers)} inliers): '
            f'pos=({x:.4f}, {y:.4f}, {z:.4f}) '
            f'quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})'
        )

        return H_grasp, last_H_to, last_H_tb

    def _call(self, client, command):
        req = JsonCommand.Request()
        req.command = json.dumps(command)
        self.get_logger().info(f'Sending: {req.command}')
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        if future.result() is not None:
            resp = json.loads(future.result().response)
            self.get_logger().info(f'Result: {resp["message"]}')
            return resp['success']
        self.get_logger().error('Service call failed')
        return False

    def move_pose(self, pose_name, speed=1.0):
        return self._call(self._joint_client, {'pose': pose_name, 'speed': speed})

    def move_cartesian(self, x, y, z, qx, qy, qz, qw, speed=0.2, keep_orientation=False, straight_line = False):
        return self._call(self._cartesian_client, {
            'x': x, 'y': y, 'z': z,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'speed': speed,
            'keep_orientation': keep_orientation,
            'straight_line': straight_line
        })

    def open_gripper(self):
        return self._call(self._gripper_client, {'action': 'open'})

    def close_gripper(self):
        return self._call(self._gripper_client, {'action': 'close'})

    def move_to_pre_grasp(self, H_target, speed=0.1, keep_orientation=False, straight_line=False):
        """Compute grasp pose from visible tags and move there."""
        self.get_logger().info('Computing grasp pose from visible AprilTags...')
        # H_target = self.compute_grasp_pose()
        if H_target is None:
            return False

        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_target)
        self.get_logger().info(
            f'Target TCP in link_base: '
            f'pos=({x:.4f}, {y:.4f}, {z:.4f}) '
            f'quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})'
        )

        return self.move_cartesian(x, y, z+0.05, qx, qy, qz, qw, speed, keep_orientation, straight_line)

    def move_to_grasp(self, H_target, speed=0.1, keep_orientation=False, straight_line=False):
        """Compute grasp pose from visible tags and move there."""
        self.get_logger().info('Computing grasp pose from visible AprilTags...')
        # H_target = self.compute_grasp_pose()
        if H_target is None:
            return False

        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_target)
        self.get_logger().info(
            f'Target TCP in link_base: '
            f'pos=({x:.4f}, {y:.4f}, {z:.4f}) '
            f'quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})'
        )

        return self.move_cartesian(x, y, z, qx, qy, qz, qw, speed, keep_orientation, straight_line)

    def _get_current_tcp_pose(self):
        """Read current TCP pose from world model. Returns [x,y,z,qx,qy,qz,qw] or None."""
        rclpy.spin_once(self, timeout_sec=0.5)
        if self._world_model is None:
            return None
        tcp = self._world_model.get('tcp_pose')
        if tcp is None:
            return None
        p = tcp['position']
        o = tcp['orientation']
        return [p['x'], p['y'], p['z'], o['qx'], o['qy'], o['qz'], o['qw']]
    
    def _lookup_tf_as_H(self, parent_frame, child_frame):
        """Look up a TF transform and return as 4x4 homogeneous matrix, or None."""
        try:
            tf = self._tf_buffer.lookup_transform(parent_frame, child_frame, Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            H = np.eye(4)
            H[:3, :3] = quat_to_rotation_matrix(r.x, r.y, r.z, r.w)
            H[:3, 3] = [t.x, t.y, t.z]
            return H
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup {parent_frame} → {child_frame} failed: {e}')
            return None

    def move_to_look_at_tags(self, standoff=0.25, height_offset=0.07, speed=0.1):
        """Move the arm so the camera looks at the AprilTags head-on.

        1. Gets tag poses from world model
        2. Computes desired camera_color_optical_frame pose (looking at tags)
        3. Uses TF to get camera_color_optical_frame → link_tcp transform
        4. Converts to desired link_tcp pose in link_base and moves there

        Returns True on success, False otherwise.
        """
        H_to = self._get_tag_H('tag_object')
        H_tb = self._get_tag_H('tag_base')

        # Gather available tag positions and rotations
        positions = []
        rotations = []
        if H_to is not None:
            positions.append(H_to[:3, 3])
            rotations.append(H_to[:3, :3])
        if H_tb is not None:
            positions.append(H_tb[:3, 3])
            rotations.append(H_tb[:3, :3])

        if not positions:
            self.get_logger().error('No AprilTags visible — cannot compute look-at pose')
            return False

        tag_position = np.mean(positions, axis=0)
        tag_rotation = rotations[0]

        # Desired camera_color_optical_frame pose in link_base
        H_co_base_desired = compute_look_at_camera_pose(
            tag_position, tag_rotation, standoff, height_offset
        )

        # Get the fixed transform: camera_color_optical_frame → link_tcp via TF
        H_tcp_co = self._lookup_tf_as_H('camera_color_optical_frame', 'link_tcp')
        if H_tcp_co is None:
            self.get_logger().error('Cannot look up camera_color_optical_frame → link_tcp TF')
            return False

        # Desired TCP pose in link_base:
        # H_tcp_base = H_co_base_desired @ H_tcp_co
        H_tcp_base = H_co_base_desired @ H_tcp_co

        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_tcp_base)
        self.get_logger().info(
            f'Look-at TCP pose: pos=({x:.4f}, {y:.4f}, {z:.4f}) '
            f'quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})'
        )

        return self.move_cartesian(x, y, z, qx, qy, qz, qw, speed=speed)

    def activate_guide_mode(self, mode):

        return self._call(self._guide_mode_client, {'action': mode, 'sensitivity': 5})

    def _log_grasp_data(self, H_to, H_tb, H_grasp, actual_tcp_pose):
        """Append one JSON record to grasp_log.jsonl."""
        log_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'logs'
        )
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, 'grasp_log.jsonl')

        # Determine run_id from existing lines
        run_id = 0
        if os.path.exists(log_path):
            with open(log_path, 'r') as f:
                run_id = sum(1 for _ in f)

        def h_to_list(H):
            if H is None:
                return None
            x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H)
            return [round(float(v), 6) for v in [x, y, z, qx, qy, qz, qw]]

        record = {
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'run_id': run_id,
            'tag_object_pose': h_to_list(H_to),
            'tag_base_pose': h_to_list(H_tb),
            'computed_grasp_pose': h_to_list(H_grasp),
            'actual_tcp_pose': [round(float(v), 6) for v in actual_tcp_pose] if actual_tcp_pose else None,
        }

        with open(log_path, 'a') as f:
            f.write(json.dumps(record) + '\n')

        self.get_logger().info(f'Logged grasp data (run {run_id}) to {log_path}')


def main():
    rclpy.init()
    node = AprilTagGraspNode()
    node.wait_for_services()

    if not node.wait_for_world_model():
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info('=== Starting AprilTag grasp sequence ===')
    # node.open_gripper()
    time.sleep(1)
    # node.close_gripper()
    # time.sleep(1)
    # node.open_gripper()

    # # node.open_gripper()
    for i in range(1):

        # 1. Home and open gripper
        node.move_pose('home', speed=0.2)
        time.sleep(0.5)
        # node.open_gripper()
        # node.move_cartesian(0.383682, 2.2e-05, 0.143111, 0.973144, 5e-06, -0.230197, -4e-06, speed=0.2, keep_orientation=False)
        # time.sleep(1)

        # Move camera to look at tags head-on for better pose estimation
        node.get_logger().info('--- Moving to look at tags head-on ---')
        if not node.move_to_look_at_tags():
            node.get_logger().warn('Failed to move to look-at pose, continuing anyway')
        time.sleep(1)

        H_target, H_to, H_tb = node.compute_grasp_pose()
        time.sleep(1)

        # 2. Move to grasp pose computed from visible tag(s)
        node.get_logger().info('--- Moving to AprilTag grasp pose ---')
        if node.move_to_pre_grasp(H_target, speed=0.1):
            # time.sleep(0.5)
            # node.close_gripper()
            time.sleep(0.5)
        else:
            node.get_logger().warn('Failed to reach pre-grasp pose')

        if node.move_to_grasp(H_target, speed=0.01, keep_orientation = True):
            # time.sleep(0.5)
            # node.close_gripper()
            time.sleep(0.5)
        else:
            node.get_logger().warn('Failed to reach grasp pose')

        # node.activate_guide_mode('enable')

        # node.close_gripper()
        to_move = input("type anything to continue")

        # Log grasp data after reaching grasp pose
        # time.sleep(1)  # let arm settle
        # actual_tcp = node._get_current_tcp_pose()
        # node._log_grasp_data(H_to, H_tb, H_target, actual_tcp)

        # node.activate_guide_mode('disable')
        time.sleep(0.5)

        # Move 5cm above current TCP pose (arm may have shifted during guide mode)
        current_tcp = node._get_current_tcp_pose()
        if current_tcp is not None:
            cx, cy, cz, cqx, cqy, cqz, cqw = current_tcp
            if node.move_cartesian(cx, cy, cz + 0.05, cqx, cqy, cqz, cqw, speed=0.01, keep_orientation=True, straight_line=True):
                time.sleep(0.5)
            else:
                node.get_logger().warn('Failed to reach post-grasp retreat pose')
        else:
            node.get_logger().warn('Could not read current TCP pose for retreat')

        # 3. Return home
        node.move_pose('home', speed=0.2)
        time.sleep(1)
        # node.open_gripper()
        time.sleep(0.5)

    node.get_logger().info('=== AprilTag grasp sequence {i} complete ===')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
