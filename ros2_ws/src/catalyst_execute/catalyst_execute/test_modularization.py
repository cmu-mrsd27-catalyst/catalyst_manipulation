#!/usr/bin/env python3
"""Test script for the modularized pick-and-place pipeline.

Uses the new service-based architecture:
  - /joint_command, /cartesian_command (MoveIt via motion_planner)
  - /gripper_command (gripper_node)
  - /guide_mode (motion_planner)
  - /set_octomap_enabled, /clear_octomap (move_group)
  - /explore_tag (explore_tag_service)
  - /sdk_control (sdk_control_service)
  - pose_math utils for pose computation

Flow:
  1. Home → close gripper
  2. Explore for AprilTag
  3. Compute pick/place/approach poses
  4. Pick from stand 1 (MoveIt + gripper + guide mode)
  5. Home
  6. Place on stand 2 (MoveIt approach → SDK corner registration →
     SDK position correction → SDK force descent → gripper open)
  7. SDK cleanup → restart controllers
  8. Retract → home

Usage:
  ros2 run catalyst_execute test_modularization
"""

import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from catalyst_interfaces.srv import JsonCommand

from catalyst_execute.utils.pose_math import (
    compute_approach_pose,
    compute_pose_from_tag,
    homogeneous_to_pos_quat,
    pose_to_homogeneous,
)

JOINT_SERVICE = '/joint_command'
CARTESIAN_SERVICE = '/cartesian_command'
GRIPPER_SERVICE = '/gripper_command'
GUIDE_MODE_SERVICE = '/guide_mode'
EXPLORE_TAG_SERVICE = '/explore_tag'
SDK_CONTROL_SERVICE = '/sdk_control'

PRE_HEIGHT = 0.05  # 5cm above pick/place pose


class TestModularization(Node):
    def __init__(self):
        super().__init__('test_modularization')

        # Load calibrated transforms
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
            config = json.load(f)
        self._H_TCP_stand1 = np.array(config['H_TCP_TO_stand1'])
        self._H_TCP_stand2 = np.array(config['H_TCP_TO_stand2'])
        self.get_logger().info(f'Loaded transforms from {config_path}')

        # World model (for reading current TCP pose)
        self._world_model = None
        self.create_subscription(String, '/world_model', self._world_model_cb, 10)

        # Service clients
        self._joint_client = self.create_client(JsonCommand, JOINT_SERVICE)
        self._cartesian_client = self.create_client(JsonCommand, CARTESIAN_SERVICE)
        self._gripper_client = self.create_client(JsonCommand, GRIPPER_SERVICE)
        self._guide_mode_client = self.create_client(JsonCommand, GUIDE_MODE_SERVICE)
        # self._explore_client = self.create_client(JsonCommand, EXPLORE_TAG_SERVICE)
        self._sdk_client = self.create_client(JsonCommand, SDK_CONTROL_SERVICE)
        self._clear_octomap_client = self.create_client(Empty, '/clear_octomap')
        self._set_octomap_client = self.create_client(SetBool, '/set_octomap_enabled')

    def _world_model_cb(self, msg: String):
        self._world_model = json.loads(msg.data)

    # ── Generic service call helpers ──

    def _call_json(self, client, command, timeout=60.0):
        """Call a JsonCommand service and return parsed response dict."""
        req = JsonCommand.Request()
        req.command = json.dumps(command)
        self.get_logger().info(f'→ {client.srv_name}: {command}')
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error(f'Service call to {client.srv_name} failed')
            return {'success': False, 'message': 'Service call failed'}
        resp = json.loads(future.result().response)
        self.get_logger().info(f'← {resp.get("message", "")}')
        return resp

    # ── MoveIt helpers ──

    def move_pose(self, pose_name, speed=0.1):
        return self._call_json(self._joint_client, {'pose': pose_name, 'speed': speed})

    def move_cartesian(self, x, y, z, qx, qy, qz, qw, speed=0.1,
                       keep_orientation=False, straight_line=False):
        return self._call_json(self._cartesian_client, {
            'x': x, 'y': y, 'z': z,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'speed': speed,
            'keep_orientation': keep_orientation,
            'straight_line': straight_line,
        })

    # ── Gripper / Guide mode ──

    def gripper(self, action):
        return self._call_json(self._gripper_client, {'action': action})

    def guide_mode(self, action):
        return self._call_json(self._guide_mode_client, {'action': action, 'sensitivity': 5})

    # ── Octomap ──

    def clear_octomap(self):
        future = self._clear_octomap_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def set_octomap_enabled(self, enabled: bool):
        req = SetBool.Request()
        req.data = enabled
        future = self._set_octomap_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    # ── SDK control ──

    def sdk_call(self, command, timeout=120.0):
        return self._call_json(self._sdk_client, command, timeout=timeout)

    # ── TCP pose from world model ──

    def get_current_tcp_pose(self):
        rclpy.spin_once(self, timeout_sec=0.5)
        if self._world_model is None:
            return None
        tcp = self._world_model.get('tcp_pose')
        if tcp is None:
            return None
        p = tcp['position']
        o = tcp['orientation']
        return [p['x'], p['y'], p['z'], o['qx'], o['qy'], o['qz'], o['qw']]

    # ── Wait for services ──

    def wait_for_services(self, timeout=None):
        self.get_logger().info('Waiting for services...')
        self._joint_client.wait_for_service(timeout_sec=timeout)
        self._cartesian_client.wait_for_service(timeout_sec=timeout)
        self._gripper_client.wait_for_service(timeout_sec=timeout)
        self._guide_mode_client.wait_for_service(timeout_sec=timeout)
        self._sdk_client.wait_for_service(timeout_sec=timeout)
        self.get_logger().info('All services ready.')

    def wait_for_world_model(self, timeout=10.0):
        self.get_logger().info('Waiting for world model...')
        start = time.time()
        while self._world_model is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('Timed out waiting for /world_model')
                return False
        return True


def main():
    rclpy.init()
    node = TestModularization()

    try:
        node.wait_for_services()
        if not node.wait_for_world_model():
            return

        # ========== 1. Home ==========
        node.get_logger().info('========== Home ==========')
        node.set_octomap_enabled(True)
        node.move_pose('home')
        node.clear_octomap()
        time.sleep(0.5)
        node.gripper('close')
        time.sleep(0.5)

        # ========== 2. Hardcoded AprilTag pose (skip exploration) ==========
        tag_pose = {
            'position': {'x': -0.321512, 'y': 0.590565, 'z': 0.079846},
            'orientation': {'qx': -0.039334, 'qy': 0.706857, 'qz': -0.706159, 'qw': -0.012113},
        }
        node.get_logger().info('Using hardcoded tag pose (exploration skipped)')

        # ========== 3. Compute poses ==========
        H_pick = compute_pose_from_tag(tag_pose, node._H_TCP_stand1)
        H_place = compute_pose_from_tag(tag_pose, node._H_TCP_stand2)
        H_tag = pose_to_homogeneous(tag_pose['position'], tag_pose['orientation'])
        H_approach_pick = compute_approach_pose(H_pick, H_tag)
        H_approach_place = compute_approach_pose(H_place, H_tag)

        px, py, pz, pqx, pqy, pqz, pqw = homogeneous_to_pos_quat(H_pick)
        lx, ly, lz, lqx, lqy, lqz, lqw = homogeneous_to_pos_quat(H_place)
        apx, apy, apz, apqx, apqy, apqz, apqw = homogeneous_to_pos_quat(H_approach_pick)
        alx, aly, alz, alqx, alqy, alqz, alqw = homogeneous_to_pos_quat(H_approach_place)

        node.get_logger().info(f'Pick:  ({px:.4f}, {py:.4f}, {pz:.4f})')
        node.get_logger().info(f'Place: ({lx:.4f}, {ly:.4f}, {lz:.4f})')

        # ========== 4. Pick from stand 1 ==========
        node.get_logger().info('========== Approach pick ==========')
        node.set_octomap_enabled(True)
        resp = node.move_cartesian(apx, apy, apz + PRE_HEIGHT, apqx, apqy, apqz, apqw)
        if not resp.get('success'):
            node.get_logger().error('Failed to reach pick approach')
            return
        time.sleep(0.5)

        node.get_logger().info('========== Pre-pick ==========')
        resp = node.move_cartesian(px, py, pz + PRE_HEIGHT, pqx, pqy, pqz, pqw)
        if not resp.get('success'):
            node.get_logger().error('Failed to reach pre-pick')
            return
        time.sleep(0.5)

        node.get_logger().info('========== Open gripper ==========')
        node.gripper('open')
        time.sleep(0.5)

        node.get_logger().info('========== Pick ==========')
        node.set_octomap_enabled(False)
        resp = node.move_cartesian(px, py, pz, pqx, pqy, pqz, pqw, speed=0.01)
        if not resp.get('success'):
            node.get_logger().error('Failed to reach pick pose')
            return
        time.sleep(0.5)

        # node.get_logger().info('========== Close gripper ==========')
        # node.gripper('close')
        # time.sleep(1.0)

        node.get_logger().info('========== Guide mode — verify grasp ==========')
        node.guide_mode('enable')
        input('Press Enter to continue...')
        node.guide_mode('disable')
        time.sleep(0.5)

        # Retract from pick (use current TCP in case user adjusted in guide mode)
        current_tcp = node.get_current_tcp_pose()
        if current_tcp is not None:
            cx, cy, cz, cqx, cqy, cqz, cqw = current_tcp
            node.get_logger().info('========== Retract from pick ==========')
            node.move_cartesian(cx, cy, cz + PRE_HEIGHT, cqx, cqy, cqz, cqw,
                                speed=0.01, keep_orientation=True, straight_line=True)
        else:
            node.move_cartesian(px, py, pz + PRE_HEIGHT, pqx, pqy, pqz, pqw,
                                speed=0.01, keep_orientation=True, straight_line=True)
        time.sleep(0.5)
        node.set_octomap_enabled(True)

        # Retreat to approach
        node.get_logger().info('========== Retreat to approach ==========')
        node.move_cartesian(apx, apy, apz, apqx, apqy, apqz, apqw, keep_orientation=True)
        time.sleep(0.5)

        # ========== 5. Home ==========
        node.get_logger().info('========== Home ==========')
        node.move_pose('home')
        time.sleep(0.5)

        # ========== 6. Place on stand 2 (corner registration) ==========

        # Offset x,y from TCP pose that lands near one corner of the stand
        offset_x = 0.117472
        offset_y = 0.778676
        node.get_logger().info(
            f'Using offset pre-place: x={offset_x:.4f}, y={offset_y:.4f} '
            f'(normal: x={lx:.4f}, y={ly:.4f})'
        )

        # Approach place at offset position
        node.get_logger().info('========== Approach place (offset) ==========')
        node.set_octomap_enabled(True)
        resp = node.move_cartesian(offset_x, offset_y, lz + PRE_HEIGHT,
                                   lqx, lqy, lqz, lqw)
        if not resp.get('success'):
            node.get_logger().error('Failed to reach place approach')
            return
        time.sleep(0.5)

        # Pre-place at offset position
        node.get_logger().info('========== Pre-place (offset) ==========')
        resp = node.move_cartesian(offset_x, offset_y, lz + PRE_HEIGHT,
                                   lqx, lqy, lqz, lqw)
        if not resp.get('success'):
            node.get_logger().error('Failed to reach pre-place')
            return
        time.sleep(1.0)
        node.set_octomap_enabled(False)

        # Connect SDK + F/T sensor
        node.get_logger().info('========== SDK connect ==========')
        resp = node.sdk_call({'action': 'connect'})
        if not resp.get('success'):
            node.get_logger().error(f'SDK connect failed: {resp.get("message")}')
            return

        # Corner registration
        node.get_logger().info('========== Corner registration ==========')
        resp = node.sdk_call({
            'action': 'velocity_control',
            'mode': 'corner_registration',
            'ref_velocity': -0.005,
            'contact_force_threshold': 1.0,
            'corner_force_threshold': 1.0,
            'search_speed': 0.003,
            'max_distance': 0.1,
        })

        if resp.get('success') and resp.get('corner_pose'):
            corner = resp['corner_pose']
            node.get_logger().info(
                f'Corner found at: X={corner[0]:.2f}, Y={corner[1]:.2f}, Z={corner[2]:.2f} mm'
            )

            # Read current position and compute corrected pose
            pos_resp = node.sdk_call({'action': 'get_position'})
            if not pos_resp.get('success'):
                node.get_logger().error('Cannot read position after corner registration')
                node.sdk_call({'action': 'cleanup'})
                return
            cur = pos_resp['position']

            # Lift up and shift to corrected position
            corrected_x = cur[0] - 5.0   # mm
            corrected_y = cur[1] + 5.0   # mm
            corrected_z = cur[2] + 5.0   # mm

            node.get_logger().info('========== Position correction ==========')
            resp = node.sdk_call({
                'action': 'position_move',
                'x': corrected_x,
                'y': corrected_y,
                'z': corrected_z,
                'roll': cur[3],
                'pitch': cur[4],
                'yaw': cur[5],
                'speed': 5,
                'tolerance': 0.5,
            })
            if not resp.get('success'):
                node.get_logger().warn(f'Position correction: {resp.get("message")}')

            # Force-controlled descent
            node.get_logger().info('========== Force descent ==========')
            resp = node.sdk_call({
                'action': 'velocity_control',
                'mode': 'force_descent',
                'descent_speed': -5.0,
                'force_threshold': 7.0,
                'rebias': True,
            })
            node.get_logger().info(f'Descent: {resp.get("message")}')
        else:
            node.get_logger().warn(f'Corner registration incomplete: {resp.get("message")}')

        # Open gripper to release plate
        node.get_logger().info('========== Release plate ==========')
        node.gripper('open')
        time.sleep(0.5)

        # ========== 7. Cleanup SDK and restart controllers ==========
        node.get_logger().info('========== SDK cleanup ==========')
        node.sdk_call({'action': 'cleanup'})

        node.get_logger().info('========== Restart controllers ==========')
        resp = node.sdk_call({'action': 'restart_controllers'})
        if not resp.get('success'):
            node.get_logger().error('Controller restart failed')
            return

        # Wait for MoveIt services to reconnect
        node.get_logger().info('Waiting for services after restart...')
        node.wait_for_services(timeout=30.0)
        time.sleep(1.0)

        # ========== 8. Retract and home ==========
        node.get_logger().info('========== Retract from place ==========')
        node.move_cartesian(lx, ly, lz + PRE_HEIGHT, lqx, lqy, lqz, lqw)
        time.sleep(0.5)

        node.set_octomap_enabled(True)

        node.get_logger().info('========== Retreat to approach ==========')
        node.move_cartesian(alx, aly, alz, alqx, alqy, alqz, alqw)
        time.sleep(0.5)

        node.get_logger().info('========== Home ==========')
        node.move_pose('home')

        node.get_logger().info('========== Done ==========')

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
        node.sdk_call({'action': 'cleanup'})
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
