#!/usr/bin/env python3
"""ROS 2 service node exposing AprilTag exploration via /explore_tag.

Wraps the TagExplorer class as a JsonCommand service. Sweeps joint1
through a range while checking /world_model for a target AprilTag.

Usage:
  ros2 run catalyst_execute explore_tag_service
"""

import json
import os

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from catalyst_interfaces.srv import JsonCommand
from catalyst_execute.exploration import TagExplorer, run_refine_head_on_view
from catalyst_execute.utils.execute_config import section
from catalyst_execute.utils.service_clients import RobotServiceClients

JOINT_SERVICE = '/joint_command'
EXPLORE_TAG_SERVICE = '/explore_tag'


class ExploreTagService(Node):
    def __init__(self):
        super().__init__('explore_tag_service')
        self._cb_group = ReentrantCallbackGroup()
        cfg = section('explore_action_server')
        self._refine_head_on = bool(cfg.get('refine_head_on_after_coarse', True))
        self._refine_distance_m = float(cfg.get('refine_standoff_distance_m', 0.4))
        self._refine_axis_sign = float(cfg.get('refine_tag_axis_sign', -1.0))
        wu = cfg.get('refine_world_up', [0.0, 0.0, 1.0])
        if isinstance(wu, (list, tuple)) and len(wu) >= 3:
            self._refine_world_up = [float(wu[0]), float(wu[1]), float(wu[2])]
        else:
            self._refine_world_up = [0.0, 0.0, 1.0]
        self._refine_move_speed = float(cfg.get('refine_move_speed', 0.25))
        self._refine_settle_sec = float(cfg.get('refine_settle_sec', 0.6))
        self._refine_cartesian_timeout_sec = float(
            cfg.get('refine_cartesian_timeout_sec', 60.0))
        self._refine_fresh_read_timeout_sec = float(
            cfg.get('refine_fresh_read_timeout_sec', 2.0))
        self._refine_service_wait_sec = float(
            cfg.get('refine_service_wait_sec', 10.0))
        self._refine_tool_axis_toward_tag = cfg.get(
            'refine_tool_axis_toward_tag', 'plus_x')
        self._refine_post_rotate_tcp_x_deg = float(
            cfg.get('refine_post_rotate_tcp_x_deg', 0.0))
        self._testing_override = bool(
            cfg.get('testing_override_tag_pose_after_explore', False))

        # World model subscription
        self._world_model = None
        self._world_model_ts = None
        self.create_subscription(
            String, '/world_model', self._world_model_cb, 10,
            callback_group=self._cb_group)

        # Joint command client (used by TagExplorer)
        self._joint_client = self.create_client(
            JsonCommand, JOINT_SERVICE, callback_group=self._cb_group)
        self._svc = RobotServiceClients(self, self._cb_group)

        self.create_service(
            JsonCommand, EXPLORE_TAG_SERVICE, self._handle_request,
            callback_group=self._cb_group)
        self.get_logger().info(f'Explore tag service ready on {EXPLORE_TAG_SERVICE}')

    def _world_model_cb(self, msg: String):
        data = json.loads(msg.data)
        self._world_model_ts = data.get('timestamp')
        self._world_model = data

    def _get_world_model(self):
        return (self._world_model_ts, self._world_model)

    def _handle_request(self, request, response):
        try:
            cmd = json.loads(request.command)
        except json.JSONDecodeError as e:
            response.response = json.dumps({
                'success': False, 'message': f'Invalid JSON: {e}'
            })
            return response

        # Wait for joint command service
        if not self._joint_client.wait_for_service(timeout_sec=5.0):
            response.response = json.dumps({
                'success': False, 'message': f'{JOINT_SERVICE} service not available'
            })
            return response

        tag_frame = cmd.get('tag_frame', 'tag_liquid_handler')
        sweep_start = cmd.get('sweep_start')
        sweep_end = cmd.get('sweep_end')
        sweep_step = cmd.get('sweep_step')
        move_speed = cmd.get('move_speed', 0.1)

        kwargs = {
            'tag_frame': tag_frame,
            'move_speed': move_speed,
        }
        if sweep_start is not None:
            kwargs['sweep_start'] = sweep_start
        if sweep_end is not None:
            kwargs['sweep_end'] = sweep_end
        if sweep_step is not None:
            kwargs['sweep_step'] = sweep_step

        explorer = TagExplorer(
            node=self,
            joint_client=self._joint_client,
            world_model_getter=self._get_world_model,
            **kwargs,
        )

        tag_pose, j1_angle = explorer.search()

        if tag_pose is not None:
            msg = f'Tag found at j1={j1_angle:.1f} deg'
            do_refine = self._refine_head_on
            if 'refine_head_on' in cmd:
                do_refine = bool(cmd['refine_head_on'])
            if do_refine and not self._testing_override:
                if self._svc.wait_for_services(
                        names=['cartesian', 'set_octomap'],
                        timeout=self._refine_service_wait_sec):
                    tag_pose = run_refine_head_on_view(
                        self,
                        self._svc,
                        self._get_world_model,
                        tag_frame,
                        tag_pose,
                        distance_m=self._refine_distance_m,
                        axis_sign=self._refine_axis_sign,
                        move_speed=float(
                            cmd.get('refine_move_speed', self._refine_move_speed)),
                        settle_sec=self._refine_settle_sec,
                        cartesian_timeout_sec=self._refine_cartesian_timeout_sec,
                        fresh_read_timeout_sec=self._refine_fresh_read_timeout_sec,
                        world_up=tuple(self._refine_world_up),
                        tool_axis_toward_tag=str(cmd.get(
                            'refine_tool_axis_toward_tag',
                            self._refine_tool_axis_toward_tag,
                        )),
                        post_rotate_tcp_x_deg=float(cmd.get(
                            'refine_post_rotate_tcp_x_deg',
                            self._refine_post_rotate_tcp_x_deg,
                        )),
                    )
                    msg += '; head-on refine'
                else:
                    self.get_logger().warn(
                        'Explore tag refine skipped: cartesian or set_octomap unavailable')
            response.response = json.dumps({
                'success': True,
                'message': msg,
                'tag_pose': tag_pose,
                'j1_angle': j1_angle,
            })
        else:
            response.response = json.dumps({
                'success': False,
                'message': f'Tag "{tag_frame}" not found in sweep range',
            })

        return response


def main():
    rclpy.init()
    node = ExploreTagService()
    n_threads = max(8, (os.cpu_count() or 4) * 2)
    executor = MultiThreadedExecutor(num_threads=n_threads)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
