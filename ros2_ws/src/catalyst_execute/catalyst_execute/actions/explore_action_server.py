#!/usr/bin/env python3
"""Explore action server.

Sweeps joint1 through a range while monitoring /world_model for an
AprilTag detection. Wraps the existing TagExplorer class as an action.

Goal (JSON):
  tag_frame:   string (default "tag_liquid_handler")
  sweep_start: float (optional, degrees)
  sweep_end:   float (optional, degrees)
  sweep_step:  float (optional, degrees)
  move_speed:  float (default from YAML, typically 0.1)
  refine_head_on: bool (optional) — override YAML coarse→head-on refine after sweep
  refine_tool_axis_toward_tag: string (optional) — e.g. plus_x, minus_z (see YAML)
  refine_post_rotate_tcp_x_deg: float (optional) — extra TCP-local +X rotation (deg)

Result (JSON):
  success:    bool
  tag_pose:   {position: {x,y,z}, orientation: {qx,qy,qz,qw}} or null
  j1_angle:   float (degrees where tag was found)
  message:    string

Optional (catalyst_execute_params.yaml → explore_action_server):
  testing_override_tag_pose_after_explore: true — after a successful sweep,
  replace tag_pose with testing_tag_pose_in_base (fixed w.r.t. link_base).

Feedback (JSON):
  phase, message

Usage:
  ros2 run catalyst_execute explore_action_server
"""

import json
import os

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from catalyst_interfaces.action import ExecuteTask
from catalyst_interfaces.srv import JsonCommand
from catalyst_execute.exploration import TagExplorer, run_refine_head_on_view
from catalyst_execute.utils.execute_config import section
from catalyst_execute.utils.service_clients import RobotServiceClients


class ExploreActionServer(Node):
    def __init__(self):
        super().__init__('explore_action_server')
        cfg = section('explore_action_server')
        self._action_name = cfg.get('action_name', '/explore')
        self._world_model_topic = cfg.get('world_model_topic', '/world_model')
        self._joint_command_service = cfg.get(
            'joint_command_service', '/joint_command')
        self._robot_phase_topic = cfg.get('robot_phase_topic', '/robot_phase')
        self._world_model_queue_size = int(cfg.get('world_model_queue_size', 10))
        self._phase_pub_queue_size = int(cfg.get('phase_pub_queue_size', 10))
        self._joint_service_wait_sec = float(
            cfg.get('joint_service_wait_sec', 10.0))
        self._default_tag_frame = cfg.get(
            'default_tag_frame', 'tag_liquid_handler')
        self._default_move_speed = float(cfg.get('default_move_speed', 0.1))
        self._testing_override_tag_pose = bool(
            cfg.get('testing_override_tag_pose_after_explore', False))
        self._testing_tag_pose_in_base = cfg.get('testing_tag_pose_in_base')

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

        self._cb_group = ReentrantCallbackGroup()

        # World model subscription
        self._world_model = None
        self._world_model_ts = None
        self.create_subscription(
            String, self._world_model_topic, self._world_model_cb,
            self._world_model_queue_size,
            callback_group=self._cb_group)

        # Joint command client (used by TagExplorer)
        self._joint_client = self.create_client(
            JsonCommand, self._joint_command_service,
            callback_group=self._cb_group)

        self._svc = RobotServiceClients(self, self._cb_group)

        self._phase_pub = self.create_publisher(
            String, self._robot_phase_topic, self._phase_pub_queue_size)
        self._publish_phase('IDLE')

        self._action_server = ActionServer(
            self, ExecuteTask, self._action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f'Explore action server ready on {self._action_name}')

    def _publish_phase(self, phase):
        msg = String()
        msg.data = phase
        self._phase_pub.publish(msg)

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Explore cancel requested')
        return CancelResponse.ACCEPT

    def _world_model_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._world_model_ts = data.get('timestamp')
        self._world_model = data

    def _get_world_model(self):
        return (self._world_model_ts, self._world_model)

    def _execute_cb(self, goal_handle):
        try:
            cmd = json.loads(goal_handle.request.command)
        except json.JSONDecodeError as e:
            goal_handle.abort()
            result = ExecuteTask.Result()
            result.response = json.dumps({
                'success': False, 'message': str(e)})
            return result

        # Wait for joint command service
        if not self._joint_client.wait_for_service(
                timeout_sec=self._joint_service_wait_sec):
            goal_handle.abort()
            result = ExecuteTask.Result()
            result.response = json.dumps({
                'success': False,
                'message': f'{self._joint_command_service} not available'})
            return result

        # Parse parameters (goal overrides YAML defaults)
        tag_frame = cmd.get('tag_frame', self._default_tag_frame)
        kwargs = {
            'tag_frame': tag_frame,
            'move_speed': cmd.get('move_speed', self._default_move_speed),
        }
        if 'sweep_start' in cmd:
            kwargs['sweep_start'] = cmd['sweep_start']
        if 'sweep_end' in cmd:
            kwargs['sweep_end'] = cmd['sweep_end']
        if 'sweep_step' in cmd:
            kwargs['sweep_step'] = cmd['sweep_step']

        # Feedback: starting exploration
        self._publish_phase('EXPLORING')
        fb = ExecuteTask.Feedback()
        fb.feedback = json.dumps({
            'phase': 'EXPLORING',
            'message': f'Searching for {tag_frame}',
        })
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'[Explore] Searching for {tag_frame}')

        explorer = TagExplorer(
            node=self,
            joint_client=self._joint_client,
            world_model_getter=self._get_world_model,
            **kwargs,
        )
        tag_pose, j1_angle = explorer.search()

        result = ExecuteTask.Result()
        if tag_pose is not None:
            msg_suffix = f'Tag found at j1={j1_angle:.1f} deg'
            do_refine = self._refine_head_on
            if 'refine_head_on' in cmd:
                do_refine = bool(cmd['refine_head_on'])
            if do_refine and not self._testing_override_tag_pose:
                self._publish_phase('REFINING')
                fb2 = ExecuteTask.Feedback()
                fb2.feedback = json.dumps({
                    'phase': 'REFINING',
                    'message': f'Head-on view ~{self._refine_distance_m:.2f} m for {tag_frame}',
                })
                goal_handle.publish_feedback(fb2)
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
                    msg_suffix += '; head-on refine'
                else:
                    self.get_logger().warn(
                        '[Explore] Refine skipped: /cartesian_command or '
                        '/set_octomap_enabled not available')

            if self._testing_override_tag_pose:
                tp = self._testing_tag_pose_in_base
                if isinstance(tp, dict) and 'position' in tp and 'orientation' in tp:
                    tag_pose = {
                        'position': dict(tp['position']),
                        'orientation': dict(tp['orientation']),
                    }
                    msg_suffix += ' (TESTING: tag_pose overridden from YAML)'
                    self.get_logger().warn(
                        '[Explore] testing_override_tag_pose_after_explore: '
                        'using testing_tag_pose_in_base for pick/place'
                    )
                else:
                    self.get_logger().error(
                        '[Explore] testing_override set but '
                        'testing_tag_pose_in_base missing/invalid — keeping vision pose'
                    )
            goal_handle.succeed()
            result.response = json.dumps({
                'success': True,
                'tag_pose': tag_pose,
                'j1_angle': j1_angle,
                'message': msg_suffix,
            })
        else:
            goal_handle.abort()
            result.response = json.dumps({
                'success': False,
                'tag_pose': None,
                'j1_angle': 0.0,
                'message': f'Tag "{tag_frame}" not found in sweep range',
            })
        self._publish_phase('IDLE')
        return result


def main():
    rclpy.init()
    node = ExploreActionServer()
    # Enough threads to run action server tasks, subscriptions, and service clients
    # concurrently while TagExplorer waits without nested spin_* (see exploration.py).
    n_threads = max(8, (os.cpu_count() or 4) * 2)
    executor = MultiThreadedExecutor(num_threads=n_threads)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
