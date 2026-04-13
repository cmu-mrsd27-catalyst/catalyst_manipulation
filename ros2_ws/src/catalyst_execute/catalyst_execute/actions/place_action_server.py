#!/usr/bin/env python3
"""Place action server.

Handles the full place sequence:
  1. Compute poses from tag
  2. Approach place at offset (octomap on, collision sensitivity 5)
  3. Pre-place at offset (still 5); then octomap off
  4. SDK connect (collision sensitivity 0)
  5. Corner registration (velocity control)
  6. Position correction (SDK position move: ΔZ then ΔX+ΔY when staged_place_position_correction)
  7. Force descent (velocity control with F/T)
  8. Open gripper — release object
  9. SDK position move +Z (clear vertically; default 50 mm) while still on SDK
  10. SDK cleanup + restart ros2_control controllers
  11. MoveIt cartesian to approach_place_offset_pre (octomap on), then joint home

Leaves the arm at home. Failure recovery still uses a two-step retract
(pre_place_offset → approach_place_offset_pre) without the SDK Z lift.

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
import os
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String as StringMsg
from catalyst_interfaces.action import ExecuteTask
from catalyst_execute.actions.robot_container_utils import (
    sdk_z_delta_position_move_cmd,
)
from catalyst_execute.utils.execute_config import section
from catalyst_execute.utils.service_clients import RobotServiceClients
from catalyst_execute.utils.world_model_cache import WorldModelCache


class PlaceActionServer(Node):
    def __init__(self):
        super().__init__('place_action_server')
        cfg = section('place_action_server')
        self._action_name = cfg.get('action_name', '/place')
        self._robot_phase_topic = cfg.get('robot_phase_topic', '/robot_phase')
        self._phase_pub_queue_size = int(cfg.get('phase_pub_queue_size', 10))
        self._default_contact_force = float(
            cfg.get('default_contact_force_threshold', 1.0))
        self._default_corner_force = float(
            cfg.get('default_corner_force_threshold', 1.0))
        self._default_descent_force = float(
            cfg.get('default_descent_force_threshold', 5.0))
        self._default_descent_speed = float(
            cfg.get('default_descent_speed', -5.0))
        self._default_search_speed = float(
            cfg.get('default_search_speed', 0.003))
        self._default_ref_velocity = float(
            cfg.get('default_ref_velocity', -0.005))
        self._sdk_cleanup_timeout_sec = float(
            cfg.get('sdk_cleanup_timeout_sec', 30.0))
        self._sdk_connect_timeout_sec = float(
            cfg.get('sdk_connect_timeout_sec', 30.0))
        self._corner_registration_timeout_sec = float(
            cfg.get('corner_registration_timeout_sec', 120.0))
        self._position_correction_timeout_sec = float(
            cfg.get('position_correction_timeout_sec', 30.0))
        self._force_descent_timeout_sec = float(
            cfg.get('force_descent_timeout_sec', 60.0))
        self._restart_controllers_timeout_sec = float(
            cfg.get('restart_controllers_timeout_sec', 60.0))
        self._wait_moveit_services_timeout_sec = float(
            cfg.get('wait_moveit_services_timeout_sec', 30.0))
        self._move_cartesian_timeout_sec = float(
            cfg.get('move_cartesian_timeout_sec', 60.0))
        self._motion_settle_sec = float(
            cfg.get(
                'motion_settle_sec',
                cfg.get(
                    'sleep_after_pre_place_sec',
                    cfg.get('sleep_after_approach_sec', 0.5),
                ),
            ))
        self._sleep_after_release_sec = float(
            cfg.get('sleep_after_release_sec', 0.5))
        self._sleep_after_restart_sec = float(
            cfg.get('sleep_after_restart_sec', 1.0))
        self._corner_registration_max_distance = float(
            cfg.get('corner_registration_max_distance', 0.1))
        self._start_settle_sec = float(cfg.get('start_settle_sec', 0.8))
        self._sdk_post_release_z_lift_mm = float(
            cfg.get('sdk_post_release_z_lift_mm', 50.0))
        self._sdk_z_lift_speed_mm_s = float(
            cfg.get('sdk_pre_place_retract_speed_mm_s', 30.0))
        self._sdk_z_lift_tolerance_mm = float(
            cfg.get('sdk_pre_place_retract_tolerance_mm', 1.0))
        self._sdk_z_lift_timeout_sec = float(
            cfg.get('sdk_pre_place_retract_timeout_sec', 120.0))
        self._joint_home_speed = float(cfg.get('joint_home_speed', 0.1))

        self._cb_group = ReentrantCallbackGroup()
        self._svc = RobotServiceClients(self, self._cb_group)

        self._phase_pub = self.create_publisher(
            StringMsg, self._robot_phase_topic, self._phase_pub_queue_size)
        self._publish_phase('IDLE')

        self._action_server = ActionServer(
            self, ExecuteTask, self._action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self._wmc = WorldModelCache(self, self._cb_group)
        self.get_logger().info(f'Place action server ready on {self._action_name}')

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Place cancel requested')
        return CancelResponse.ACCEPT

    # ── Helpers ──

    def _publish_phase(self, phase):
        msg = StringMsg()
        msg.data = phase
        self._phase_pub.publish(msg)

    def _feedback(self, goal_handle, phase, message=''):
        self._publish_phase(phase)
        fb = ExecuteTask.Feedback()
        fb.feedback = json.dumps({'phase': phase, 'message': message})
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'[Place] {phase}: {message}')

    def _result(self, success, error_code='', message=''):
        self._publish_phase('IDLE')
        r = ExecuteTask.Result()
        r.response = json.dumps({
            'success': success,
            'error_code': error_code,
            'message': message,
        })
        return r

    def _canceled(self, goal_handle):
        if goal_handle.is_cancel_requested:
            self.get_logger().warn('[Place] Cancel detected — cleaning up')
            goal_handle.canceled()
            return True
        return False

    def _sdk_cleanup(self):
        """Best-effort SDK cleanup — always safe to call."""
        self.get_logger().info('Running SDK cleanup...')
        self._svc.sdk_call(
            {'action': 'cleanup'}, timeout=self._sdk_cleanup_timeout_sec)

    def _after_motion(self):
        """Pause after each cartesian execute or SDK arm-motion segment."""
        if self._motion_settle_sec > 0.0:
            time.sleep(self._motion_settle_sec)

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
        contact_force = cmd.get(
            'contact_force_threshold', self._default_contact_force)
        corner_force = cmd.get(
            'corner_force_threshold', self._default_corner_force)
        descent_force = cmd.get(
            'descent_force_threshold', self._default_descent_force)
        descent_speed = cmd.get('descent_speed', self._default_descent_speed)
        search_speed = cmd.get('search_speed', self._default_search_speed)
        ref_velocity = cmd.get('ref_velocity', self._default_ref_velocity)

        # Wait for required services
        needed = [
            'cartesian', 'gripper', 'compute_poses', 'sdk', 'octomap',
            'clear_octomap',
        ]
        if not self._svc.wait_for_services(names=needed):
            goal_handle.abort()
            return self._result(False, 'SERVICES_UNAVAILABLE',
                                'Required services not available')

        if self._start_settle_sec > 0.0:
            self._feedback(
                goal_handle, 'SETTLING',
                f'Waiting {self._start_settle_sec:.1f}s for arm to settle')
            time.sleep(self._start_settle_sec)

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

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

        # Long approach from explore/home: octomap off for transit (see pick_action_server).
        self._svc.set_octomap_enabled(False)

        # --- 1. Compute all poses from tag ---
        self._feedback(goal_handle, 'COMPUTING_POSES')
        resp = self._svc.compute_poses({'tag_pose': tag_pose})
        if not resp.get('success'):
            goal_handle.abort()
            return self._result(False, 'COMPUTE_POSES_FAILED',
                                resp.get('message', ''))

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 2. Approach place at offset (transit: octomap off) ---
        self._feedback(goal_handle, 'APPROACHING', 'Moving to approach place')

        pose_cmd = self._svc.compute_poses(
            {'query': 'approach_place_offset_pre'})
        if not pose_cmd.get('success'):
            goal_handle.abort()
            return self._result(False, 'POSE_QUERY_FAILED',
                                pose_cmd.get('message', ''))

        move = self._svc.move_cartesian_cmd(
            pose_cmd, timeout=self._move_cartesian_timeout_sec)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'APPROACH_FAILED',
                                move.get('message', ''))
        self._after_motion()

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        self._svc.set_octomap_enabled(True)
        self._svc.clear_octomap()

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 3. Pre-place at offset position ---
        self._feedback(goal_handle, 'PRE_PLACING', 'Moving to pre-place')
        pose_cmd = self._svc.compute_poses({'query': 'pre_place_offset'})
        if not pose_cmd.get('success'):
            goal_handle.abort()
            return self._result(False, 'POSE_QUERY_FAILED',
                                pose_cmd.get('message', ''))

        move = self._svc.move_cartesian_cmd(
            pose_cmd, timeout=self._move_cartesian_timeout_sec)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'PRE_PLACE_FAILED',
                                move.get('message', ''))
        self._after_motion()

        self._svc.set_octomap_enabled(False)

        # Enable EEF bounds for close-range work
        self._svc.set_eef_bounds(tag_pose)

        if self._canceled(goal_handle):
            self._svc.clear_eef_bounds()
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 4. SDK connect (close-range force work) ---
        self._feedback(goal_handle, 'SDK_CONNECTING',
                       'Connecting to xArm SDK + F/T sensor')
        resp = self._svc.sdk_call(
            {'action': 'connect'}, timeout=self._sdk_connect_timeout_sec)
        if not resp.get('success'):
            goal_handle.abort()
            return self._result(False, 'SDK_CONNECT_FAILED',
                                resp.get('message', ''))
        sdk_connected = True

        if self._canceled(goal_handle):
            self._sdk_cleanup()
            self._restart_and_retract(goal_handle)
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
            'max_distance': self._corner_registration_max_distance,
        }, timeout=self._corner_registration_timeout_sec)

        if not resp.get('success'):
            self._feedback(goal_handle, 'CORNER_REGISTRATION',
                           f'Corner registration failed: {resp.get("message", "")}')
            self._sdk_cleanup()
            self._restart_and_retract(goal_handle)
            goal_handle.abort()
            return self._result(False, 'CORNER_REGISTRATION_FAILED',
                                resp.get('message', ''))

        self._after_motion()

        if self._canceled(goal_handle):
            self._sdk_cleanup()
            self._restart_and_retract(goal_handle)
            return self._result(False, 'CANCELED', 'Canceled')

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

        moves = corrected.get('correction_moves')
        if moves:
            _stage_labels = ('Z offset (lift)', 'X+Y offset')
            for i, mv in enumerate(moves):
                label = (
                    _stage_labels[i] if i < len(_stage_labels)
                    else f'segment {i + 1}')
                self._feedback(
                    goal_handle, 'POSITION_CORRECTION',
                    f'Stage {i + 1}/{len(moves)}: {label}')
                move = self._svc.sdk_call(
                    mv, timeout=self._position_correction_timeout_sec)
                if not move.get('success'):
                    self._feedback(goal_handle, 'POSITION_CORRECTION',
                                   f'Move failed: {move.get("message", "")}')
                    self._sdk_cleanup()
                    self._restart_and_retract(goal_handle)
                    goal_handle.abort()
                    return self._result(False, 'POSITION_CORRECTION_FAILED',
                                        move.get('message', ''))
                self._after_motion()
        else:
            move = self._svc.sdk_call(
                corrected, timeout=self._position_correction_timeout_sec)
            if not move.get('success'):
                self._feedback(goal_handle, 'POSITION_CORRECTION',
                               f'Move failed: {move.get("message", "")}')
                self._sdk_cleanup()
                self._restart_and_retract(goal_handle)
                goal_handle.abort()
                return self._result(False, 'POSITION_CORRECTION_FAILED',
                                    move.get('message', ''))
            self._after_motion()

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
        }, timeout=self._force_descent_timeout_sec)
        self._feedback(goal_handle, 'FORCE_DESCENT',
                       resp.get('message', ''))
        self._after_motion()

        if self._canceled(goal_handle):
            self._sdk_cleanup()
            self._restart_and_retract(goal_handle)
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 8. Release object ---
        self._feedback(goal_handle, 'RELEASING', 'Opening gripper')
        self._wmc.gripper_open_if_needed(self._svc)
        time.sleep(self._sleep_after_release_sec)

        # --- 9. SDK vertical lift before leaving SDK mode ---
        self._feedback(
            goal_handle, 'SDK_LIFT',
            f'SDK position move +Z {self._sdk_post_release_z_lift_mm:.0f} mm')
        pos_resp = self._svc.sdk_call({'action': 'get_position'})
        if not pos_resp.get('success'):
            self.get_logger().warn(
                f'SDK get_position before Z lift: {pos_resp.get("message", "")}')
        else:
            z_lift_cmd = sdk_z_delta_position_move_cmd(
                pos_resp['position'],
                self._sdk_post_release_z_lift_mm,
                speed_mm_s=self._sdk_z_lift_speed_mm_s,
                tolerance_mm=self._sdk_z_lift_tolerance_mm,
            )
            move = self._svc.sdk_call(
                z_lift_cmd, timeout=self._sdk_z_lift_timeout_sec)
            if not move.get('success'):
                self.get_logger().warn(
                    f'SDK Z lift after release: {move.get("message", "")}')
        self._after_motion()

        # --- 10. SDK cleanup + restart controllers ---
        self._feedback(goal_handle, 'CLEANUP',
                       'SDK cleanup and controller restart')
        self._sdk_cleanup()

        resp = self._svc.sdk_call(
            {'action': 'restart_controllers'},
            timeout=self._restart_controllers_timeout_sec)
        if not resp.get('success'):
            goal_handle.abort()
            return self._result(False, 'CONTROLLER_RESTART_FAILED',
                                resp.get('message', ''))

        # Wait for MoveIt services to reconnect
        self._feedback(goal_handle, 'WAITING_FOR_SERVICES',
                       'Waiting for controllers to recover')
        self._svc.wait_for_services(
            names=['joint', 'cartesian'],
            timeout=self._wait_moveit_services_timeout_sec)
        time.sleep(self._sleep_after_restart_sec)

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 11. MoveIt to approach (offset) then joint home ---
        self._svc.clear_eef_bounds()
        self._retract_to_approach_place_offset(goal_handle)

        self._feedback(goal_handle, 'HOME', 'Joint home')
        move = self._svc.move_home(speed=self._joint_home_speed)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'HOME_FAILED', move.get('message', ''))
        self._after_motion()

        goal_handle.succeed()
        return self._result(True, '', 'Place complete')

    def _retract_to_approach_place_offset(self, goal_handle):
        """Single MoveIt segment to approach_place_offset_pre (octomap on)."""
        self._feedback(
            goal_handle, 'RETRACTING',
            'MoveIt to approach place (offset)')
        self._svc.set_octomap_enabled(True)

        pose_cmd = self._svc.compute_poses(
            {'query': 'approach_place_offset_pre'})
        if pose_cmd.get('success'):
            self._svc.move_cartesian_cmd(
                pose_cmd, timeout=self._move_cartesian_timeout_sec)
        self._after_motion()

    def _retract(self, goal_handle):
        """Failure recovery: pre-place offset then approach (octomap on)."""
        self._feedback(goal_handle, 'RETRACTING', 'Retracting from place')

        pose_cmd = self._svc.compute_poses({'query': 'pre_place_offset'})
        if pose_cmd.get('success'):
            self._svc.move_cartesian_cmd(
                pose_cmd, timeout=self._move_cartesian_timeout_sec)
        self._after_motion()

        self._svc.set_octomap_enabled(True)

        pose_cmd = self._svc.compute_poses(
            {'query': 'approach_place_offset_pre'})
        if pose_cmd.get('success'):
            self._svc.move_cartesian_cmd(
                pose_cmd, timeout=self._move_cartesian_timeout_sec)
        self._after_motion()

    def _restart_and_retract(self, goal_handle):
        """Restart controllers and retract — used on failure paths."""
        self._feedback(goal_handle, 'RECOVERY',
                       'Restarting controllers after failure')
        try:
            self._svc.clear_eef_bounds()
        except Exception:
            pass
        resp = self._svc.sdk_call(
            {'action': 'restart_controllers'},
            timeout=self._restart_controllers_timeout_sec)
        if resp.get('success'):
            self._svc.wait_for_services(
                names=['joint', 'cartesian'],
                timeout=self._wait_moveit_services_timeout_sec)
            time.sleep(self._sleep_after_restart_sec)
            self._retract(goal_handle)


def main():
    rclpy.init()
    node = PlaceActionServer()
    n_threads = max(8, (os.cpu_count() or 4) * 2)
    executor = MultiThreadedExecutor(num_threads=n_threads)
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
