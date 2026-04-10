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
from catalyst_execute.utils.execute_config import section
from catalyst_execute.utils.pose_math import (
    compute_approach_pose,
    compute_pose_from_tag,
    homogeneous_to_pos_quat,
    pose_to_homogeneous,
)


class ComputePosesService(Node):
    def __init__(self):
        super().__init__('compute_poses_service')

        cp = section('compute_poses')
        self._service_name = cp.get('service_name', '/compute_poses')
        self._pre_height = float(cp.get('pre_height', 0.05))
        self._pick_grasp_offset_z_m = float(
            cp.get('pick_grasp_offset_z_m', 0.0))
        self._pre_place_planar_dx = float(
            cp.get('pre_place_planar_offset_x_m', 0.015))
        self._pre_place_planar_dy = float(
            cp.get('pre_place_planar_offset_y_m', -0.015))
        self._correction_dx = float(cp.get('correction_dx', -3.0))
        self._correction_dy = float(cp.get('correction_dy', 3.0))
        self._correction_dz = float(cp.get('correction_dz', 5.0))
        self._default_cartesian_speed = float(
            cp.get('default_cartesian_speed', 0.1))
        self._position_move_speed = float(cp.get('position_move_speed', 5))
        self._position_move_tolerance = float(
            cp.get('position_move_tolerance', 0.5))
        self._approach_tag_y_offset_m = float(
            cp.get('approach_tag_y_offset_m', -0.05))

        gripper_file = cp.get('gripper_pose_file', 'gripper_pose.json')
        # services/ -> inner package dir -> ament package root (config/)
        _here = os.path.dirname(os.path.abspath(__file__))
        pkg_root = os.path.normpath(os.path.join(_here, '..', '..'))
        config_path = os.path.join(pkg_root, 'config', gripper_file)
        if not os.path.exists(config_path):
            config_path = os.path.join(
                get_package_share_directory('catalyst_execute'),
                'config', gripper_file,
            )
        with open(config_path, 'r') as f:
            config = json.load(f)
        self._H_TCP_stand1 = np.array(config['H_TCP_TO_stand1'])
        self._H_TCP_stand2 = np.array(config['H_TCP_TO_stand2'])
        self.get_logger().info(f'Loaded transforms from {config_path}')

        self._poses = {}

        self.create_service(JsonCommand, self._service_name, self._handle_request)
        self.get_logger().info(
            f'Compute poses service ready on {self._service_name}')

    def _handle_request(self, request, response):
        try:
            cmd = json.loads(request.command)
        except json.JSONDecodeError as e:
            response.response = json.dumps({
                'success': False, 'message': f'Invalid JSON: {e}'
            })
            return response

        if 'tag_pose' in cmd:
            tag_pose = cmd['tag_pose']
            self._compute_all(tag_pose)
            response.response = json.dumps({
                'success': True,
                'message': 'Poses computed',
                'poses': self._poses,
            })
            return response

        if 'corner_pose' in cmd:
            result = self._compute_corrected_pose(cmd)
            response.response = json.dumps(result)
            return response

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
        H_pick[2, 3] += self._pick_grasp_offset_z_m
        H_place = compute_pose_from_tag(tag_pose, self._H_TCP_stand2)
        tag_pos = tag_pose['position']
        ap_kw = dict(
            tag_position=tag_pos,
            pre_height_m=self._pre_height,
            approach_tag_y_offset_m=self._approach_tag_y_offset_m,
        )
        H_approach_pick = compute_approach_pose(H_pick, **ap_kw)
        H_approach_place = compute_approach_pose(H_place, **ap_kw)

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
            return self._make_cmd(p, z_offset=self._pre_height)
        elif query == 'approach_pick':
            return self._make_cmd(ap)
        elif query == 'approach_pick_pre':
            return self._make_cmd(ap, z_offset=self._pre_height)
        elif query == 'pre_place':
            return self._make_cmd(l, z_offset=self._pre_height)
        elif query == 'approach_place':
            return self._make_cmd(al)
        elif query == 'pre_place_offset':
            lx, ly = l[0], l[1]
            return self._make_cmd(
                l,
                x_override=lx + self._pre_place_planar_dx,
                y_override=ly + self._pre_place_planar_dy,
                z_offset=self._pre_height)
        elif query == 'approach_place_offset_pre':
            return self._make_cmd(
                al,
                x_override=al[0] + self._pre_place_planar_dx,
                y_override=al[1] + self._pre_place_planar_dy,
                z_offset=self._pre_height)
        else:
            return None

    def _make_cmd(self, pose, speed=None, z_offset=0.0,
                  keep_orientation=False, straight_line=False,
                  x_override=None, y_override=None):
        """Build a /cartesian_command JSON from a 7-element pose."""
        if speed is None:
            speed = self._default_cartesian_speed
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

        return {
            'success': True,
            'message': 'Corrected pose computed',
            'action': 'position_move',
            'x': corner_pose[0] + self._correction_dx,
            'y': corner_pose[1] + self._correction_dy,
            'z': corner_pose[2] + self._correction_dz,
            'roll': corner_pose[3],
            'pitch': corner_pose[4],
            'yaw': corner_pose[5],
            'speed': self._position_move_speed,
            'tolerance': self._position_move_tolerance,
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
