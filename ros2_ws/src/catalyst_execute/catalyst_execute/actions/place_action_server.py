#!/usr/bin/env python3
"""Place action server.

Handles the full place sequence:
  1. Compute poses from tag
  2. Approach place at offset position (octomap on)
  3. Pre-place at offset position
  4. SDK connect (xArm SDK + F/T sensor)
  5. Corner registration (velocity control)
  6. Position correction (SDK position move)
  7. Force descent (velocity control with F/T)
  8. Open gripper — release object
  9. SDK cleanup + restart ros2_control controllers
  10. Retract to pre-place → approach (octomap on)

Leaves the arm at the approach_place pose.

Goal (JSON):
  tag_pose: {position: {x,y,z}, orientation: {qx,qy,qz,qw}}
  contact_force_threshold: float (default 1.0 N)
  corner_force_threshold:  float (default 1.0 N)
  descent_force_threshold: float (default 5.0 N)
  descent_speed:           float (default -5.0 mm/s)
  search_speed:            float (default 0.003 m/s)
  ref_velocity:            float (default -0.005 m/s)

Result (JSON):
  success, error_code, message

Feedback (JSON):
  phase, message

Usage:
  ros2 run catalyst_execute place_action_server
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


class PlaceActionServer(Node):
    def __init__(self):
        super().__init__('place_action_server')
        self._cb_group = ReentrantCallbackGroup()
        self._svc = RobotServiceClients(self, self._cb_group)

        self._action_server = ActionServer(
            self, ExecuteTask, '/place',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info('Place action server ready on /place')

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Place cancel requested')
        return CancelResponse.ACCEPT

    # ── Helpers ──

    def _feedback(self, goal_handle, phase, message=''):
        fb = ExecuteTask.Feedback()
        fb.feedback = json.dumps({'phase': phase, 'message': message})
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'[Place] {phase}: {message}')

    def _result(self, success, error_code='', message=''):
        r = ExecuteTask.Result()
        r.response = json.dumps({
            'success': success,
            'error_code': error_code,
            'message': message,
        })
        return r

    def _canceled(self, goal_handle):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return True
        return False

    def _sdk_cleanup(self):
        """Best-effort SDK cleanup — always safe to call."""
        self.get_logger().info('Running SDK cleanup...')
        self._svc.sdk_call({'action': 'cleanup'}, timeout=30.0)

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

        # Placement parameters (with defaults matching current behavior)
        contact_force = cmd.get('contact_force_threshold', 1.0)
        corner_force = cmd.get('corner_force_threshold', 1.0)
        descent_force = cmd.get('descent_force_threshold', 5.0)
        descent_speed = cmd.get('descent_speed', -5.0)
        search_speed = cmd.get('search_speed', 0.003)
        ref_velocity = cmd.get('ref_velocity', -0.005)

        # Wait for required services
        needed = ['cartesian', 'gripper', 'compute_poses', 'sdk', 'octomap']
        if not self._svc.wait_for_services(names=needed):
            goal_handle.abort()
            return self._result(False, 'SERVICES_UNAVAILABLE',
                                'Required services not available')

        sdk_connected = False

        try:
            return self._run_place(
                goal_handle, tag_pose,
                contact_force, corner_force, descent_force,
                descent_speed, search_speed, ref_velocity,
                sdk_connected,
            )
        except Exception as e:
            self.get_logger().error(f'Place failed with exception: {e}')
            self._sdk_cleanup()
            goal_handle.abort()
            return self._result(False, 'EXCEPTION', str(e))

    def _run_place(self, goal_handle, tag_pose,
                   contact_force, corner_force, descent_force,
                   descent_speed, search_speed, ref_velocity,
                   sdk_connected):

        # --- 1. Compute all poses from tag ---
        self._feedback(goal_handle, 'COMPUTING_POSES')
        resp = self._svc.compute_poses({'tag_pose': tag_pose})
        if not resp.get('success'):
            goal_handle.abort()
            return self._result(False, 'COMPUTE_POSES_FAILED',
                                resp.get('message', ''))

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 2. Approach place at offset position (octomap on) ---
        self._feedback(goal_handle, 'APPROACHING', 'Moving to approach place')
        self._svc.set_octomap_enabled(True)

        pose_cmd = self._svc.compute_poses(
            {'query': 'approach_place_offset_pre'})
        if not pose_cmd.get('success'):
            goal_handle.abort()
            return self._result(False, 'POSE_QUERY_FAILED',
                                pose_cmd.get('message', ''))

        move = self._svc.move_cartesian_cmd(pose_cmd, timeout=60.0)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'APPROACH_FAILED',
                                move.get('message', ''))
        time.sleep(0.5)

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 3. Pre-place at offset position ---
        self._feedback(goal_handle, 'PRE_PLACING', 'Moving to pre-place')
        pose_cmd = self._svc.compute_poses({'query': 'pre_place_offset'})
        if not pose_cmd.get('success'):
            goal_handle.abort()
            return self._result(False, 'POSE_QUERY_FAILED',
                                pose_cmd.get('message', ''))

        move = self._svc.move_cartesian_cmd(pose_cmd, timeout=60.0)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'PRE_PLACE_FAILED',
                                move.get('message', ''))
        time.sleep(1.0)

        self._svc.set_octomap_enabled(False)

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 4. SDK connect ---
        self._feedback(goal_handle, 'SDK_CONNECTING',
                       'Connecting to xArm SDK + F/T sensor')
        resp = self._svc.sdk_call({'action': 'connect'}, timeout=30.0)
        if not resp.get('success'):
            goal_handle.abort()
            return self._result(False, 'SDK_CONNECT_FAILED',
                                resp.get('message', ''))
        sdk_connected = True

        if self._canceled(goal_handle):
            self._sdk_cleanup()
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 5. Corner registration ---
        self._feedback(goal_handle, 'CORNER_REGISTRATION',
                       'Running corner registration')
        resp = self._svc.sdk_call({
            'action': 'velocity_control',
            'mode': 'corner_registration',
            'ref_velocity': ref_velocity,
            'contact_force_threshold': contact_force,
            'corner_force_threshold': corner_force,
            'search_speed': search_speed,
            'max_distance': 0.1,
        }, timeout=120.0)

        if not resp.get('success'):
            self._feedback(goal_handle, 'CORNER_REGISTRATION',
                           f'Corner registration failed: {resp.get("message", "")}')
            self._sdk_cleanup()
            self._restart_and_retract(goal_handle)
            goal_handle.abort()
            return self._result(False, 'CORNER_REGISTRATION_FAILED',
                                resp.get('message', ''))

        # --- 6. Position correction ---
        self._feedback(goal_handle, 'POSITION_CORRECTION',
                       'Computing and applying position correction')
        corrected = self._svc.compute_poses(resp)
        if not corrected.get('success'):
            self._feedback(goal_handle, 'POSITION_CORRECTION',
                           f'Correction failed: {corrected.get("message", "")}')
            self._sdk_cleanup()
            self._restart_and_retract(goal_handle)
            goal_handle.abort()
            return self._result(False, 'POSITION_CORRECTION_FAILED',
                                corrected.get('message', ''))

        move = self._svc.sdk_call(corrected, timeout=30.0)
        if not move.get('success'):
            self.get_logger().warn(
                f'Position correction move: {move.get("message", "")}')

        if self._canceled(goal_handle):
            self._sdk_cleanup()
            self._restart_and_retract(goal_handle)
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 7. Force descent ---
        self._feedback(goal_handle, 'FORCE_DESCENT',
                       f'Force descent at {descent_speed} mm/s')
        resp = self._svc.sdk_call({
            'action': 'velocity_control',
            'mode': 'force_descent',
            'descent_speed': descent_speed,
            'force_threshold': descent_force,
            'rebias': True,
        }, timeout=60.0)
        self._feedback(goal_handle, 'FORCE_DESCENT',
                       resp.get('message', ''))

        # --- 8. Release object ---
        self._feedback(goal_handle, 'RELEASING', 'Opening gripper')
        self._svc.gripper('open')
        time.sleep(0.5)

        # --- 9. SDK cleanup + restart controllers ---
        self._feedback(goal_handle, 'CLEANUP',
                       'SDK cleanup and controller restart')
        self._sdk_cleanup()

        resp = self._svc.sdk_call(
            {'action': 'restart_controllers'}, timeout=60.0)
        if not resp.get('success'):
            goal_handle.abort()
            return self._result(False, 'CONTROLLER_RESTART_FAILED',
                                resp.get('message', ''))

        # Wait for MoveIt services to reconnect
        self._feedback(goal_handle, 'WAITING_FOR_SERVICES',
                       'Waiting for controllers to recover')
        self._svc.wait_for_services(
            names=['joint', 'cartesian'], timeout=30.0)
        time.sleep(1.0)

        # --- 10. Retract ---
        self._retract(goal_handle)

        goal_handle.succeed()
        return self._result(True, '', 'Place complete')

    def _retract(self, goal_handle):
        """Retract to pre-place then approach-place (octomap on)."""
        self._feedback(goal_handle, 'RETRACTING', 'Retracting from place')

        pose_cmd = self._svc.compute_poses({'query': 'pre_place'})
        if pose_cmd.get('success'):
            self._svc.move_cartesian_cmd(pose_cmd, timeout=60.0)
        time.sleep(0.5)

        self._svc.set_octomap_enabled(True)

        pose_cmd = self._svc.compute_poses({'query': 'approach_place'})
        if pose_cmd.get('success'):
            self._svc.move_cartesian_cmd(pose_cmd, timeout=60.0)
        time.sleep(0.5)

    def _restart_and_retract(self, goal_handle):
        """Restart controllers and retract — used on failure paths."""
        self._feedback(goal_handle, 'RECOVERY',
                       'Restarting controllers after failure')
        resp = self._svc.sdk_call(
            {'action': 'restart_controllers'}, timeout=60.0)
        if resp.get('success'):
            self._svc.wait_for_services(
                names=['joint', 'cartesian'], timeout=30.0)
            time.sleep(1.0)
            self._retract(goal_handle)


def main():
    rclpy.init()
    node = PlaceActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node._sdk_cleanup()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
