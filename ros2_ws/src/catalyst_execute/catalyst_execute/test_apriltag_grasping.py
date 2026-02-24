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

from std_msgs.msg import String
from catalyst_interfaces.srv import JsonCommand

JOINT_SERVICE = '/joint_command'
CARTESIAN_SERVICE = '/cartesian_command'
GRIPPER_SERVICE = '/gripper_command'


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
        self._H_TCP_TB = np.array(grasp_data['H_TCP_TB'])  # TCP w.r.t. tag_base
        self.get_logger().info(f'Loaded grasp transforms from {config_path}')

        # World model state
        self._world_model = None
        self.create_subscription(String, '/world_model', self._world_model_cb, 10)

        # Service clients
        self._joint_client = self.create_client(JsonCommand, JOINT_SERVICE)
        self._cartesian_client = self.create_client(JsonCommand, CARTESIAN_SERVICE)
        self._gripper_client = self.create_client(JsonCommand, GRIPPER_SERVICE)

    def _world_model_cb(self, msg: String):
        self._world_model = json.loads(msg.data)

    def wait_for_services(self):
        self.get_logger().info('Waiting for motion planner services...')
        self._joint_client.wait_for_service()
        self._cartesian_client.wait_for_service()
        self._gripper_client.wait_for_service()
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

    def compute_grasp_pose(self):
        """Compute target TCP pose in link_base using visible AprilTags.

        H_TCP_base = H_tag_base @ H_TCP_tag

        If both tags visible, averages the two resulting TCP poses.
        If only one visible, uses that one.

        Returns:
            (H_grasp, H_to, H_tb) — grasp pose and raw tag poses (each may be None)
        """
        H_to = self._get_tag_H('tag_object')
        H_tb = self._get_tag_H('tag_base')

        H_from_to = None
        H_from_tb = None

        if H_to is not None:
            H_from_to = H_to @ self._H_TCP_TO
            self.get_logger().info('tag_object visible — computed grasp pose from it')

        if H_tb is not None:
            H_from_tb = H_tb @ self._H_TCP_TB
            self.get_logger().info('tag_base visible — computed grasp pose from it')

        if H_from_to is not None and H_from_tb is not None:
            self.get_logger().info('Both tags visible — averaging grasp poses')
            return average_poses(H_from_to, H_from_tb), H_to, H_tb
        elif H_from_to is not None:
            return H_from_to, H_to, H_tb
        elif H_from_tb is not None:
            return H_from_tb, H_to, H_tb
        else:
            self.get_logger().error('No AprilTags visible — cannot compute grasp pose')
            return None, H_to, H_tb

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

    def move_cartesian(self, x, y, z, qx, qy, qz, qw, speed=0.2, keep_orientation=False):
        return self._call(self._cartesian_client, {
            'x': x, 'y': y, 'z': z,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'speed': speed,
            'keep_orientation': keep_orientation
        })

    def open_gripper(self):
        return self._call(self._gripper_client, {'action': 'open'})

    def close_gripper(self):
        return self._call(self._gripper_client, {'action': 'close'})

    def move_to_pre_grasp(self, H_target, speed=0.1, keep_orientation=False):
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

        return self.move_cartesian(x, y, z+0.05, qx, qy, qz, qw, speed, keep_orientation)

    def move_to_grasp(self, H_target, speed=0.1, keep_orientation=False):
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

        return self.move_cartesian(x, y, z, qx, qy, qz, qw, speed, keep_orientation)

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
    node.open_gripper()
    time.sleep(1)
    node.close_gripper()
    time.sleep(1)
    node.open_gripper()

    # node.open_gripper()
    # for i in range(15):

    #     # 1. Home and open gripper
    #     node.move_pose('home', speed=0.2)
    #     time.sleep(0.5)
    #     # node.open_gripper()
    #     time.sleep(1)

    #     H_target, H_to, H_tb = node.compute_grasp_pose()
    #     time.sleep(1)

    #     # 2. Move to grasp pose computed from visible tag(s)
    #     node.get_logger().info('--- Moving to AprilTag grasp pose ---')
    #     if node.move_to_pre_grasp(H_target, speed=0.1, keep_orientation = True):
    #         # time.sleep(0.5)
    #         # node.close_gripper()
    #         time.sleep(0.5)
    #     else:
    #         node.get_logger().warn('Failed to reach pre-grasp pose')

    #     if node.move_to_grasp(H_target, speed=0.01, keep_orientation = True):
    #         # time.sleep(0.5)
    #         # node.close_gripper()
    #         time.sleep(0.5)
    #     else:
    #         node.get_logger().warn('Failed to reach grasp pose')

    #     # Log grasp data after reaching grasp pose
    #     time.sleep(0.5)  # let arm settle
    #     actual_tcp = node._get_current_tcp_pose()
    #     node._log_grasp_data(H_to, H_tb, H_target, actual_tcp)

    #     if node.move_to_pre_grasp(H_target, speed=0.1, keep_orientation = True):
    #         # time.sleep(0.5)
    #         # node.close_gripper()
    #         time.sleep(0.5)
    #     else:
    #         node.get_logger().warn('Failed to reach pre-grasp pose')

    #     # 3. Return home
    #     node.move_pose('home', speed=0.2)
    #     time.sleep(0.5)

    node.get_logger().info('=== AprilTag grasp sequence {i} complete ===')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
