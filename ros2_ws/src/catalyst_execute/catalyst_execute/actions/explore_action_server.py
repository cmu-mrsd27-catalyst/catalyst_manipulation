#!/usr/bin/env python3
"""Explore action server.

Sweeps joint1 through a range while monitoring /world_model for an
AprilTag detection. Wraps the existing TagExplorer class as an action.

Goal (JSON):
  tag_frame:   string (default "tag_liquid_handler")
  sweep_start: float (optional, degrees)
  sweep_end:   float (optional, degrees)
  sweep_step:  float (optional, degrees)
  move_speed:  float (default 0.3)

Result (JSON):
  success:    bool
  tag_pose:   {position: {x,y,z}, orientation: {qx,qy,qz,qw}} or null
  j1_angle:   float (degrees where tag was found)
  message:    string

Feedback (JSON):
  phase, message

Usage:
  ros2 run catalyst_execute explore_action_server
"""

import json

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from catalyst_interfaces.action import ExecuteTask
from catalyst_interfaces.srv import JsonCommand
from catalyst_execute.exploration import TagExplorer


class ExploreActionServer(Node):
    def __init__(self):
        super().__init__('explore_action_server')
        self._cb_group = ReentrantCallbackGroup()

        # World model subscription
        self._world_model = None
        self._world_model_ts = None
        self.create_subscription(
            String, '/world_model', self._world_model_cb, 10,
            callback_group=self._cb_group)

        # Joint command client (used by TagExplorer)
        self._joint_client = self.create_client(
            JsonCommand, '/joint_command', callback_group=self._cb_group)

        self._action_server = ActionServer(
            self, ExecuteTask, '/explore',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info('Explore action server ready on /explore')

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Explore cancel requested')
        return CancelResponse.ACCEPT

    def _world_model_cb(self, msg: String):
        data = json.loads(msg.data)
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
        if not self._joint_client.wait_for_service(timeout_sec=10.0):
            goal_handle.abort()
            result = ExecuteTask.Result()
            result.response = json.dumps({
                'success': False,
                'message': '/joint_command not available'})
            return result

        # Parse parameters
        tag_frame = cmd.get('tag_frame', 'tag_liquid_handler')
        kwargs = {
            'tag_frame': tag_frame,
            'move_speed': cmd.get('move_speed', 0.3),
        }
        if 'sweep_start' in cmd:
            kwargs['sweep_start'] = cmd['sweep_start']
        if 'sweep_end' in cmd:
            kwargs['sweep_end'] = cmd['sweep_end']
        if 'sweep_step' in cmd:
            kwargs['sweep_step'] = cmd['sweep_step']

        # Feedback: starting exploration
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
            goal_handle.succeed()
            result.response = json.dumps({
                'success': True,
                'tag_pose': tag_pose,
                'j1_angle': j1_angle,
                'message': f'Tag found at j1={j1_angle:.1f} deg',
            })
        else:
            goal_handle.abort()
            result.response = json.dumps({
                'success': False,
                'tag_pose': None,
                'j1_angle': 0.0,
                'message': f'Tag "{tag_frame}" not found in sweep range',
            })
        return result


def main():
    rclpy.init()
    node = ExploreActionServer()
    executor = MultiThreadedExecutor()
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
