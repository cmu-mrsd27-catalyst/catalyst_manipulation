#!/usr/bin/env python3
"""
Test script for AprilTag-based grasping.

Flow:
  1. Explore for the liquid handler AprilTag by sweeping joint1
  2. Once found, compute the grasp pose from the tag pose and a
     pre-calibrated H_TCP_TAG transform (loaded from gripper_pose.json)
  3. Execute the grasp sequence
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
from std_srvs.srv import Empty, SetBool
from catalyst_interfaces.srv import JsonCommand

from catalyst_execute.exploration import TagExplorer

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


class AprilTagGraspNode(Node):
    def __init__(self):
        super().__init__('test_apriltag_grasping')

        # Load grasp transform from JSON
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

        # H_TCP_TAG: TCP pose relative to the liquid handler tag
        # (key is H_TCP_TO in the JSON — will be recalibrated by user)
        self._H_TCP_TAG = np.array(grasp_data['H_TCP_TO_stand1'])
        self.get_logger().info(f'Loaded grasp transform from {config_path}')

        # TF2 buffer for looking up transforms between frames
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # World model state (with timestamp tracking)
        self._world_model = None
        self._world_model_ts = None
        self.create_subscription(String, '/world_model', self._world_model_cb, 10)

        # Service clients
        self._joint_client = self.create_client(JsonCommand, JOINT_SERVICE)
        self._cartesian_client = self.create_client(JsonCommand, CARTESIAN_SERVICE)
        self._gripper_client = self.create_client(JsonCommand, GRIPPER_SERVICE)
        self._guide_mode_client = self.create_client(JsonCommand, GUIDE_MODE_SERVICE)
        self._clear_octomap_client = self.create_client(Empty, '/clear_octomap')
        self._set_octomap_client = self.create_client(SetBool, '/set_octomap_enabled')

    def _world_model_cb(self, msg: String):
        data = json.loads(msg.data)
        self._world_model_ts = data.get('timestamp')
        self._world_model = data

    def get_world_model(self):
        """Return (timestamp, world_model_dict) tuple for TagExplorer."""
        return (self._world_model_ts, self._world_model)

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

    def explore_for_tag(self):
        """Use TagExplorer to sweep joint1 and find the liquid handler tag.

        Returns:
            (tag_pose, joint1_deg) tuple. tag_pose is a dict with 'position'
            and 'orientation' in link_base, or (None, None) if not found.
        """
        explorer = TagExplorer(
            node=self,
            joint_client=self._joint_client,
            world_model_getter=self.get_world_model,
        )
        return explorer.search()

    def compute_grasp_pose(self, tag_pose):
        """Compute the grasp TCP pose from a detected tag pose.

        Args:
            tag_pose: dict with 'position' and 'orientation' keys (from world model).

        Returns:
            4x4 homogeneous matrix for the target TCP pose in link_base.
        """
        H_tag = pose_to_homogeneous(tag_pose['position'], tag_pose['orientation'])
        H_grasp = H_tag @ self._H_TCP_TAG

        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_grasp)
        self.get_logger().info(
            f'Grasp pose: pos=({x:.4f}, {y:.4f}, {z:.4f}) '
            f'quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})'
        )
        return H_grasp

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

    def clear_octomap(self):
        """Clear the MoveIt octomap so grasped objects aren't treated as obstacles."""
        self.get_logger().info('Clearing octomap...')
        future = self._clear_octomap_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.get_logger().info('Octomap cleared.')
            return True
        self.get_logger().warn('Failed to clear octomap.')
        return False

    def set_octomap_enabled(self, enabled: bool):
        """Enable or disable octomap collision checking for planning."""
        label = 'Enabling' if enabled else 'Disabling'
        self.get_logger().info(f'{label} octomap for planning...')
        req = SetBool.Request()
        req.data = enabled
        future = self._set_octomap_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.get_logger().info(f'Octomap {future.result().message}')
            return future.result().success
        self.get_logger().warn(f'Failed to set octomap enabled={enabled}')
        return False

    def move_pose(self, pose_name, speed=1.0):
        return self._call(self._joint_client, {'pose': pose_name, 'speed': speed})

    def move_cartesian(self, x, y, z, qx, qy, qz, qw, speed=0.2, keep_orientation=False, straight_line=False):
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
        """Move to 5cm above the grasp pose."""
        if H_target is None:
            return False
        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_target)
        self.get_logger().info(
            f'Pre-grasp TCP: pos=({x:.4f}, {y:.4f}, {z+0.05:.4f})'
        )
        return self.move_cartesian(x, y, z + 0.05, qx, qy, qz, qw, speed, keep_orientation, straight_line)

    def move_to_grasp(self, H_target, speed=0.1, keep_orientation=False, straight_line=False):
        """Move to the grasp pose."""
        if H_target is None:
            return False
        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_target)
        self.get_logger().info(
            f'Grasp TCP: pos=({x:.4f}, {y:.4f}, {z:.4f})'
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

    def activate_guide_mode(self, mode):
        return self._call(self._guide_mode_client, {'action': mode, 'sensitivity': 5})

    def _log_grasp_data(self, H_tag, H_grasp, actual_tcp_pose):
        """Append one JSON record to grasp_log.jsonl."""
        log_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'logs'
        )
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, 'grasp_log.jsonl')

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
            'tag_pose': h_to_list(H_tag),
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

    node.move_pose('home', speed=0.2)
    time.sleep(0.5)

    node.open_gripper()
    
    # 1. Explore for the liquid handler tag
    node.get_logger().info('--- Exploring for AprilTag ---')
    tag_pose, tag_j1 = node.explore_for_tag()
    if tag_pose is None:
        node.get_logger().error('Tag not found during exploration. Aborting.')
        node.destroy_node()
        rclpy.shutdown()
        return

    # 2. Compute grasp pose from detected tag
    H_grasp = node.compute_grasp_pose(tag_pose)

    # 3. Move to home pose with joint1 facing the machine
    node.get_logger().info(f'--- Moving to home facing machine (j1={tag_j1:.1f} deg) ---')
    node._call(node._joint_client, {
        'joints': [tag_j1, 0.0, 0.0, 0.0, 0.0, 0.0],
        'speed': 0.2,
    })
    time.sleep(0.5)

    time.sleep(1.0)

    # 4. Move to pre-grasp with octomap enabled (collision-aware)
    node.get_logger().info('--- Moving to pre-grasp pose ---')
    if node.move_to_pre_grasp(H_grasp, speed=0.1):
        time.sleep(0.5)
    else:
        node.get_logger().warn('Failed to reach pre-grasp pose')

    # 5. Disable octomap for the short pre-grasp -> grasp motion
    node.set_octomap_enabled(False)
    # node.clear_octomap()

    node.get_logger().info('--- Moving to grasp pose ---')
    if node.move_to_grasp(H_grasp, speed=0.01):
        time.sleep(0.5)
    else:
        node.get_logger().warn('Failed to reach grasp pose')

    node.close_gripper()
    time.sleep(1.0)

    # 6. Re-enable octomap so subsequent motions are collision-aware
    node.set_octomap_enabled(True)

    node.activate_guide_mode('enable')
    to_move = input("type anything to continue")
    node.activate_guide_mode('disable')

    time.sleep(0.5)

    # 4. Retreat: move 5cm above current TCP pose
    current_tcp = node._get_current_tcp_pose()
    if current_tcp is not None:
        cx, cy, cz, cqx, cqy, cqz, cqw = current_tcp
        if node.move_cartesian(cx, cy, cz + 0.05, cqx, cqy, cqz, cqw, speed=0.01, keep_orientation=True, straight_line=True):
            time.sleep(0.5)
        else:
            node.get_logger().warn('Failed to reach post-grasp retreat pose')
    else:
        node.get_logger().warn('Could not read current TCP pose for retreat')

    # # 5. Return home

    node._call(node._joint_client, {
        'joints': [tag_j1, 0.0, 0.0, 0.0, 0.0, 0.0],
        'speed': 0.1,
    })
    time.sleep(0.5)

    node.move_pose('home', speed=0.1)
    time.sleep(0.5)
    node.open_gripper()

    node.get_logger().info('=== AprilTag grasp sequence complete ===')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
