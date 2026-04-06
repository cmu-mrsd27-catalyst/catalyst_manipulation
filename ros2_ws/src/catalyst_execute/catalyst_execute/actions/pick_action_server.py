#!/usr/bin/env python3
"""Pick action server.

Handles the full pick sequence:
  1. Compute poses from tag
  2. Approach pick (octomap on)
  3. Pre-pick
  4. Open gripper
  5. Pick descent (slow, octomap off)
  6. Guide mode — manual adjustment (optional)
  7. Close gripper
  8. Retract to pre-pick → approach (octomap on)

Leaves the arm at the approach_pick pose.

Goal (JSON):
  tag_pose: {position: {x,y,z}, orientation: {qx,qy,qz,qw}}
  use_guide_mode: bool (default true)

Result (JSON):
  success, error_code, message

Feedback (JSON):
  phase, message

Usage:
  ros2 run catalyst_execute pick_action_server
"""

import json
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from catalyst_interfaces.action import ExecuteTask
from catalyst_execute.utils.service_clients import RobotServiceClients

GUIDE_MODE_SETTLE_SEC = 2.0
POST_GUIDE_SETTLE_SEC = 3.0
GUIDE_MODE_MAX_RETRIES = 5


class PickActionServer(Node):
    def __init__(self):
        super().__init__('pick_action_server')
        self._cb_group = ReentrantCallbackGroup()
        self._svc = RobotServiceClients(self, self._cb_group)

        self._action_server = ActionServer(
            self, ExecuteTask, '/pick',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info('Pick action server ready on /pick')

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Pick cancel requested')
        return CancelResponse.ACCEPT

    # ── Helpers ──

    def _feedback(self, goal_handle, phase, message=''):
        fb = ExecuteTask.Feedback()
        fb.feedback = json.dumps({'phase': phase, 'message': message})
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'[Pick] {phase}: {message}')

    def _result(self, success, error_code='', message=''):
        r = ExecuteTask.Result()
        r.response = json.dumps({
            'success': success,
            'error_code': error_code,
            'message': message,
        })
        return r

    def _canceled(self, goal_handle):
        """Check for cancellation and return True if goal was canceled."""
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return True
        return False

    # ── Execute ──

    def _execute_cb(self, goal_handle):
        try:
            cmd = json.loads(goal_handle.request.command)
        except json.JSONDecodeError as e:
            goal_handle.abort()
            return self._result(False, 'INVALID_COMMAND', str(e))

        tag_pose = cmd.get('tag_pose')
        if not tag_pose:
            goal_handle.abort()
            return self._result(False, 'MISSING_TAG_POSE', 'tag_pose required')

        use_guide_mode = cmd.get('use_guide_mode', True)
        pick_speed = cmd.get('pick_speed', 0.01)  # slow for descent/ascent

        # Wait for required services
        needed = ['cartesian', 'gripper', 'guide_mode', 'compute_poses', 'octomap']
        if not self._svc.wait_for_services(names=needed):
            goal_handle.abort()
            return self._result(False, 'SERVICES_UNAVAILABLE',
                                'Required services not available')

        # --- 1. Compute all poses from tag ---
        self._feedback(goal_handle, 'COMPUTING_POSES')
        resp = self._svc.compute_poses({'tag_pose': tag_pose})
        if not resp.get('success'):
            goal_handle.abort()
            return self._result(False, 'COMPUTE_POSES_FAILED', resp.get('message', ''))

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 2. Approach pick (with PRE_HEIGHT, octomap on) ---
        self._feedback(goal_handle, 'APPROACHING', 'Moving to approach pick')
        self._svc.set_octomap_enabled(True)

        pose_cmd = self._svc.compute_poses({'query': 'approach_pick_pre'})
        if not pose_cmd.get('success'):
            goal_handle.abort()
            return self._result(False, 'POSE_QUERY_FAILED', pose_cmd.get('message', ''))

        move = self._svc.move_cartesian_cmd(pose_cmd, timeout=60.0)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'APPROACH_FAILED', move.get('message', ''))
        time.sleep(0.5)

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 3. Pre-pick ---
        self._feedback(goal_handle, 'PRE_PICK', 'Moving to pre-pick')
        pose_cmd = self._svc.compute_poses({'query': 'pre_pick'})
        if not pose_cmd.get('success'):
            goal_handle.abort()
            return self._result(False, 'POSE_QUERY_FAILED', pose_cmd.get('message', ''))

        move = self._svc.move_cartesian_cmd(pose_cmd, timeout=60.0)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'PRE_PICK_FAILED', move.get('message', ''))
        time.sleep(0.5)

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 4. Open gripper ---
        self._feedback(goal_handle, 'OPENING_GRIPPER')
        self._svc.gripper('open')
        time.sleep(0.5)

        # --- 5. Pick descent (slow, octomap off) ---
        self._feedback(goal_handle, 'PICKING', 'Descending to pick pose')
        self._svc.set_octomap_enabled(False)

        pose_cmd = self._svc.compute_poses({'query': 'pick'})
        if not pose_cmd.get('success'):
            goal_handle.abort()
            return self._result(False, 'POSE_QUERY_FAILED', pose_cmd.get('message', ''))

        pose_cmd['speed'] = pick_speed
        move = self._svc.move_cartesian_cmd(pose_cmd, timeout=60.0)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'PICK_DESCENT_FAILED', move.get('message', ''))
        time.sleep(0.5)

        # --- 6. Guide mode (optional) ---
        if use_guide_mode:
            self._feedback(goal_handle, 'GUIDE_MODE',
                           'Guide mode enabled — adjust grasp manually')
            for attempt in range(GUIDE_MODE_MAX_RETRIES):
                resp = self._svc.guide_mode('enable')
                if resp.get('success'):
                    break
                self.get_logger().warn(
                    f'Guide mode enable attempt {attempt + 1} failed, retrying...')
                time.sleep(0.5)
            time.sleep(GUIDE_MODE_SETTLE_SEC)

        # --- 7. Close gripper ---
        self._feedback(goal_handle, 'CLOSING_GRIPPER', 'Grasping object')
        self._svc.gripper('close')
        time.sleep(1.0)

        # --- 8. Disable guide mode ---
        if use_guide_mode:
            self._feedback(goal_handle, 'DISABLING_GUIDE_MODE')
            # May report transient errors during controller reactivation — OK
            self._svc.guide_mode('disable')
            time.sleep(POST_GUIDE_SETTLE_SEC)

        # --- 9. Retract to pre-pick ---
        self._feedback(goal_handle, 'RETRACTING', 'Retracting to pre-pick')
        pose_cmd = self._svc.compute_poses({'query': 'pre_pick'})
        if pose_cmd.get('success'):
            pose_cmd['speed'] = pick_speed
            self._svc.move_cartesian_cmd(pose_cmd, timeout=60.0)
        time.sleep(0.5)

        self._svc.set_octomap_enabled(True)

        # --- 10. Retreat to approach ---
        self._feedback(goal_handle, 'RETREATING', 'Retreating to approach pose')
        pose_cmd = self._svc.compute_poses({'query': 'approach_pick'})
        if pose_cmd.get('success'):
            self._svc.move_cartesian_cmd(pose_cmd, timeout=60.0)
        time.sleep(0.5)

        goal_handle.succeed()
        return self._result(True, '', 'Pick complete')


def main():
    rclpy.init()
    node = PickActionServer()
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
