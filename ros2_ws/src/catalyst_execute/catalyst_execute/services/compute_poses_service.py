#!/usr/bin/env python3
"""ROS 2 service node exposing pose computation via /compute_poses.

Takes an AprilTag pose and returns computed pick/place/approach poses
as ready-to-send JSON commands for /cartesian_command.

Usage:
  ros2 run catalyst_execute compute_poses_service
"""

import json
import os

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from catalyst_interfaces.srv import JsonCommand
from catalyst_execute.utils.pose_math import (
    compute_approach_pose,
    compute_pose_from_tag,
    homogeneous_to_pos_quat,
    pose_to_homogeneous,
)

COMPUTE_POSES_SERVICE = '/compute_poses'
PRE_HEIGHT = 0.05  # 5cm above pick/place

# Offset position for corner registration (hits near a corner of the stand)
OFFSET_X = 0.117472
OFFSET_Y = 0.778676

# Correction offsets after finding corner (mm, applied to SDK position)
CORRECTION_DX = -5.0
CORRECTION_DY = 5.0
CORRECTION_DZ = 5.0


class ComputePosesService(Node):
    def __init__(self):
        super().__init__('compute_poses_service')

        # Load calibrated transforms
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'gripper_pose.json'
        )
        if not os.path.exists(config_path):
            config_path = os.path.join(
                get_package_share_directory('catalyst_execute'),
                'config', 'gripper_pose.json'
            )
        with open(config_path, 'r') as f:
            config = json.load(f)
        self._H_TCP_stand1 = np.array(config['H_TCP_TO_stand1'])
        self._H_TCP_stand2 = np.array(config['H_TCP_TO_stand2'])
        self.get_logger().info(f'Loaded transforms from {config_path}')

        # Cache computed poses
        self._poses = {}

        self.create_service(JsonCommand, COMPUTE_POSES_SERVICE, self._handle_request)
        self.get_logger().info(f'Compute poses service ready on {COMPUTE_POSES_SERVICE}')

    def _handle_request(self, request, response):
        try:
            cmd = json.loads(request.command)
        except json.JSONDecodeError as e:
            response.response = json.dumps({
                'success': False, 'message': f'Invalid JSON: {e}'
            })
            return response

        # If tag_pose is provided, compute and cache all poses
        if 'tag_pose' in cmd:
            tag_pose = cmd['tag_pose']
            self._compute_all(tag_pose)
            response.response = json.dumps({
                'success': True,
                'message': 'Poses computed',
                'poses': self._poses,
            })
            return response

        # If corner_pose is present, compute corrected placement pose
        if 'corner_pose' in cmd:
            result = self._compute_corrected_pose(cmd)
            response.response = json.dumps(result)
            return response

        # Query for a specific pose formatted as a /cartesian_command
        query = cmd.get('query', '')

        pose_cmd = self._get_cartesian_command(query)
        if pose_cmd is None:
            response.response = json.dumps({
                'success': False,
                'message': f'Unknown query: {query}. Compute poses first.',
            })
            return response

        response.response = json.dumps(pose_cmd)
        return response

    def _compute_all(self, tag_pose):
        """Compute and cache all poses from tag detection."""
        H_pick = compute_pose_from_tag(tag_pose, self._H_TCP_stand1)
        H_place = compute_pose_from_tag(tag_pose, self._H_TCP_stand2)
        H_tag = pose_to_homogeneous(tag_pose['position'], tag_pose['orientation'])
        H_approach_pick = compute_approach_pose(H_pick, H_tag)
        H_approach_place = compute_approach_pose(H_place, H_tag)

        px, py, pz, pqx, pqy, pqz, pqw = homogeneous_to_pos_quat(H_pick)
        lx, ly, lz, lqx, lqy, lqz, lqw = homogeneous_to_pos_quat(H_place)
        apx, apy, apz, apqx, apqy, apqz, apqw = homogeneous_to_pos_quat(H_approach_pick)
        alx, aly, alz, alqx, alqy, alqz, alqw = homogeneous_to_pos_quat(H_approach_place)

        self._poses = {
            'pick': [px, py, pz, pqx, pqy, pqz, pqw],
            'place': [lx, ly, lz, lqx, lqy, lqz, lqw],
            'approach_pick': [apx, apy, apz, apqx, apqy, apqz, apqw],
            'approach_place': [alx, aly, alz, alqx, alqy, alqz, alqw],
        }

        self.get_logger().info(
            f'Poses computed: pick=({px:.4f},{py:.4f},{pz:.4f}), '
            f'place=({lx:.4f},{ly:.4f},{lz:.4f})'
        )

    def _get_cartesian_command(self, query):
        """Return a /cartesian_command JSON for the given query."""
        if not self._poses:
            return None

        p = self._poses.get('pick')
        l = self._poses.get('place')
        ap = self._poses.get('approach_pick')
        al = self._poses.get('approach_place')

        if query == 'pick':
            return self._make_cmd(p)
        elif query == 'pre_pick':
            return self._make_cmd(p, z_offset=PRE_HEIGHT)
        elif query == 'approach_pick':
            return self._make_cmd(ap)
        elif query == 'approach_pick_pre':
            return self._make_cmd(ap, z_offset=PRE_HEIGHT)
        elif query == 'pre_place':
            return self._make_cmd(l, z_offset=PRE_HEIGHT)
        elif query == 'approach_place':
            return self._make_cmd(al)
        elif query == 'pre_place_offset':
            return self._make_cmd(l, x_override=OFFSET_X, y_override=OFFSET_Y,
                                  z_offset=PRE_HEIGHT)
        elif query == 'approach_place_offset_pre':
            return self._make_cmd(l, x_override=OFFSET_X, y_override=OFFSET_Y,
                                  z_offset=PRE_HEIGHT)
        else:
            return None

    def _make_cmd(self, pose, speed=0.1, z_offset=0.0,
                  keep_orientation=False, straight_line=False,
                  x_override=None, y_override=None):
        """Build a /cartesian_command JSON from a 7-element pose."""
        x, y, z, qx, qy, qz, qw = pose
        return {
            'success': True,
            'message': 'OK',
            'x': float(x_override if x_override is not None else x),
            'y': float(y_override if y_override is not None else y),
            'z': float(z + z_offset),
            'qx': float(qx), 'qy': float(qy),
            'qz': float(qz), 'qw': float(qw),
            'speed': speed,
            'keep_orientation': keep_orientation,
            'straight_line': straight_line,
        }

    def _compute_corrected_pose(self, cmd):
        """Compute corrected SDK position_move command from corner response."""
        corner_pose = cmd.get('corner_pose')
        if not corner_pose:
            return {'success': False, 'message': 'No corner_pose in response'}

        # Apply correction offsets (mm)
        return {
            'success': True,
            'message': 'Corrected pose computed',
            'action': 'position_move',
            'x': corner_pose[0] + CORRECTION_DX,
            'y': corner_pose[1] + CORRECTION_DY,
            'z': corner_pose[2] + CORRECTION_DZ,
            'roll': corner_pose[3],
            'pitch': corner_pose[4],
            'yaw': corner_pose[5],
            'speed': 5,
            'tolerance': 0.5,
        }


def main():
    rclpy.init()
    node = ComputePosesService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
