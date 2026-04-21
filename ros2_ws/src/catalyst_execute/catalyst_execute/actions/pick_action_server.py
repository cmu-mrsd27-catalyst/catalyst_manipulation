#!/usr/bin/env python3
"""Pick action server.

Handles the full pick sequence:
  1. Compute poses from tag
  2. Approach pick (octomap off for long transit from explore); close gripper at approach
  3. Octomap on + clear + EEF bounds; pre-pick; open gripper at pre-pick
  4. Pick descent (slow, octomap off), optional guide, close gripper
  5. If grasp verify fails (world model / finger opening), retract to pre-pick and
     repeat descent with TCP Z lowered by ``pick_z_retry_step_m`` per retry (YAML).
  6. Disable guide mode when used
  7. Retract to pre-pick → approach (octomap on)

Leaves the arm at the approach_pick pose after a successful run.

Goal (JSON):
  tag_pose: {position: {x,y,z}, orientation: {qx,qy,qz,qw}}
  use_guide_mode: bool (default true)
  grasp_verify_enabled: optional bool (overrides YAML for this goal)

Timing (catalyst_execute_params.yaml → pick_action_server):
  start_settle_sec — before first motion; motion_settle_sec — after each cartesian execute.

Result (JSON):
  success, error_code, message

Feedback (JSON):
  phase, message

Usage:
  ros2 run catalyst_execute pick_action_server
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
from catalyst_execute.utils.execute_config import section
from catalyst_execute.utils.service_clients import RobotServiceClients
from catalyst_execute.utils.world_model_cache import WorldModelCache


class PickActionServer(Node):
    def __init__(self):
        super().__init__('pick_action_server')
        cfg = section('pick_action_server')
        self._action_name = cfg.get('action_name', '/pick')
        self._robot_phase_topic = cfg.get('robot_phase_topic', '/robot_phase')
        self._phase_pub_queue_size = int(cfg.get('phase_pub_queue_size', 10))
        self._guide_mode_settle_sec = float(cfg.get('guide_mode_settle_sec', 2.0))
        self._post_guide_settle_sec = float(cfg.get('post_guide_settle_sec', 3.0))
        self._guide_mode_max_retries = int(cfg.get('guide_mode_max_retries', 5))
        self._guide_mode_retry_sleep_sec = float(
            cfg.get('guide_mode_retry_sleep_sec', 0.5))
        self._default_pick_speed = float(cfg.get('default_pick_speed', 0.01))
        self._move_timeout_sec = float(cfg.get('move_timeout_sec', 60.0))
        self._motion_settle_sec = float(
            cfg.get(
                'motion_settle_sec',
                cfg.get('sleep_after_move_sec', 0.5),
            ))
        self._sleep_after_open_gripper_sec = float(
            cfg.get('sleep_after_open_gripper_sec', 0.5))
        self._sleep_after_close_gripper_sec = float(
            cfg.get('sleep_after_close_gripper_sec', 1.0))
        self._compute_poses_timeout_sec = float(
            cfg.get('compute_poses_timeout_sec', 10.0))
        # After BT joint "home" (or any prior move), brief pause before first
        # cartesian so MoveIt start state matches the real arm.
        self._start_settle_sec = float(cfg.get('start_settle_sec', 0.8))
        self._grasp_verify_enabled = bool(cfg.get('grasp_verify_enabled', True))
        self._grasp_verify_settle_sec = float(cfg.get('grasp_verify_settle_sec', 0.2))
        self._pick_z_retry_step_m = float(cfg.get('pick_z_retry_step_m', 0.01))
        self._pick_z_retry_max_attempts = int(cfg.get('pick_z_retry_max_attempts', 3))
        raw_max_open = cfg.get('grasp_max_finger_opening_m', 0.028)
        self._grasp_max_finger_opening_m = (
            float(raw_max_open) if raw_max_open is not None else None
        )

        self._cb_group = ReentrantCallbackGroup()
        self._svc = RobotServiceClients(self, self._cb_group)

        self._action_server = ActionServer(
            self, ExecuteTask, self._action_name,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self._phase_pub = self.create_publisher(
            StringMsg, self._robot_phase_topic, self._phase_pub_queue_size)
        self._publish_phase('IDLE')
        self._wmc = WorldModelCache(self, self._cb_group)
        self.get_logger().info(f'Pick action server ready on {self._action_name}')

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Pick cancel requested')
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
        self.get_logger().info(f'[Pick] {phase}: {message}')

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
        """Check for cancellation and return True if goal was canceled."""
        if goal_handle.is_cancel_requested:
            self.get_logger().warn('[Pick] Cancel detected — cleaning up')
            goal_handle.canceled()
            return True
        return False

    def _after_motion(self):
        """Pause after each executed MoveIt trajectory so hardware / state can settle."""
        if self._motion_settle_sec > 0.0:
            time.sleep(self._motion_settle_sec)

    def _cleanup(self, guide_was_enabled):
        """Ensure safe state after cancel or failure mid-sequence."""
        self._publish_phase('RECOVERY')
        if guide_was_enabled:
            try:
                self._svc.guide_mode('disable')
                time.sleep(self._post_guide_settle_sec)
            except Exception:
                pass
        try:
            self._svc.clear_eef_bounds()
        except Exception:
            pass
        try:
            self._svc.set_octomap_enabled(True)
        except Exception:
            pass
        self._publish_phase('IDLE')

    # ── Execute ──

    def _execute_cb(self, goal_handle):
        guide_was_enabled = False

        try:
            return self._execute_inner(goal_handle)
        except Exception as e:
            self.get_logger().error(f'[Pick] Unexpected error: {e}')
            self._cleanup(guide_was_enabled)
            goal_handle.abort()
            return self._result(False, 'INTERNAL_ERROR', str(e))

    def _execute_inner(self, goal_handle):
        guide_was_enabled = False

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
        pick_speed = cmd.get('pick_speed', self._default_pick_speed)

        # Wait for required services
        needed = [
            'cartesian', 'gripper', 'guide_mode', 'compute_poses', 'octomap',
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
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        # After explore, the arm is far from the tag. A large Cartesian move with
        # octomap ON often fails (table/stand/noisy cloud or self in view). Transit
        # with octomap OFF, then enable + clear for close-range pre_pick/pick.
        # We do not call set_collision_sensitivity: it triggers xArm CONFIG_CHANGED
        # and traj controller re-arm, which breaks the following MoveIt executes.
        self._svc.set_octomap_enabled(False)

        # --- 1. Compute all poses from tag ---
        self._feedback(goal_handle, 'COMPUTING_POSES')
        resp = self._svc.compute_poses(
            {'tag_pose': tag_pose}, timeout=self._compute_poses_timeout_sec)
        if not resp.get('success'):
            goal_handle.abort()
            return self._result(False, 'COMPUTE_POSES_FAILED', resp.get('message', ''))

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 2. Approach pick (transit: octomap off) ---
        self._feedback(goal_handle, 'APPROACHING', 'Moving to approach pick')

        pose_cmd = self._svc.compute_poses(
            {'query': 'approach_pick_pre'},
            timeout=self._compute_poses_timeout_sec)
        if not pose_cmd.get('success'):
            goal_handle.abort()
            return self._result(False, 'POSE_QUERY_FAILED', pose_cmd.get('message', ''))

        move = self._svc.move_cartesian_cmd(
            pose_cmd, timeout=self._move_timeout_sec)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'APPROACH_FAILED', move.get('message', ''))
        self._after_motion()

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(
            goal_handle, 'CLOSING_GRIPPER_AT_APPROACH',
            'Close gripper at approach_pick_pre')
        self._svc.gripper('close')
        time.sleep(self._sleep_after_close_gripper_sec)

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        # Close-range segments: fresh octomap for collisions near the stand
        self._svc.set_octomap_enabled(True)
        # self._svc.clear_octomap()

        # Enable EEF bounds for close-range work (MoveIt-side)
        self._svc.set_eef_bounds(tag_pose)

        # --- 3. Pre-pick ---
        self._feedback(goal_handle, 'PRE_PICK', 'Moving to pre-pick')
        pose_cmd = self._svc.compute_poses(
            {'query': 'pre_pick'}, timeout=self._compute_poses_timeout_sec)
        if not pose_cmd.get('success'):
            goal_handle.abort()
            return self._result(False, 'POSE_QUERY_FAILED', pose_cmd.get('message', ''))

        move = self._svc.move_cartesian_cmd(
            pose_cmd, timeout=self._move_timeout_sec)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'PRE_PICK_FAILED', move.get('message', ''))
        self._after_motion()

        # Close-range segment: pre-pick → pick → grasp → retract to pre-pick (octomap off below)
        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(
            goal_handle, 'OPENING_GRIPPER',
            'Open gripper at pre-pick')
        self._svc.gripper('open')
        time.sleep(self._sleep_after_open_gripper_sec)

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 5–7. Pick descent, optional guide, close — with Z retry if grasp not verified ---
        grasp_verify = bool(
            cmd.get('grasp_verify_enabled', self._grasp_verify_enabled))
        max_open = self._grasp_max_finger_opening_m
        z_extra_m = 0.0
        max_pick_attempts = max(1, int(self._pick_z_retry_max_attempts))

        for pick_attempt in range(max_pick_attempts):
            if pick_attempt > 0:
                z_extra_m += self._pick_z_retry_step_m
                self._feedback(
                    goal_handle, 'GRASP_RETRY',
                    f'Grasp not verified — retry pick (ΔZ = −{z_extra_m * 1000:.0f} mm vs nominal), '
                    f'attempt {pick_attempt + 1}/{max_pick_attempts}')
                if guide_was_enabled:
                    self._feedback(goal_handle, 'DISABLING_GUIDE_MODE')
                    self._svc.guide_mode('disable')
                    guide_was_enabled = False
                    time.sleep(self._post_guide_settle_sec)
                self._feedback(goal_handle, 'RETRY_PRE_PICK', 'Moving to pre-pick before re-descent')
                pose_cmd = self._svc.compute_poses(
                    {'query': 'pre_pick'}, timeout=self._compute_poses_timeout_sec)
                if pose_cmd.get('success'):
                    pose_cmd['speed'] = pick_speed
                    self._svc.move_cartesian_cmd(
                        pose_cmd, timeout=self._move_timeout_sec)
                self._after_motion()
                self._feedback(goal_handle, 'OPENING_GRIPPER', 'Opening before pick retry')
                self._svc.gripper('open')
                time.sleep(self._sleep_after_open_gripper_sec)
                if self._canceled(goal_handle):
                    self._cleanup(guide_was_enabled)
                    return self._result(False, 'CANCELED', 'Canceled')

            self._feedback(goal_handle, 'PICKING', 'Descending to pick pose')
            self._svc.set_octomap_enabled(False)

            pose_cmd = self._svc.compute_poses(
                {'query': 'pick'}, timeout=self._compute_poses_timeout_sec)
            if not pose_cmd.get('success'):
                self._cleanup(guide_was_enabled)
                goal_handle.abort()
                return self._result(False, 'POSE_QUERY_FAILED', pose_cmd.get('message', ''))

            pose_cmd['z'] = float(pose_cmd['z']) - z_extra_m
            pose_cmd['speed'] = pick_speed
            move = self._svc.move_cartesian_cmd(
                pose_cmd, timeout=self._move_timeout_sec)
            if not move.get('success'):
                self._cleanup(guide_was_enabled)
                goal_handle.abort()
                return self._result(False, 'PICK_DESCENT_FAILED', move.get('message', ''))
            self._after_motion()

            if self._canceled(goal_handle):
                self._cleanup(guide_was_enabled)
                return self._result(False, 'CANCELED', 'Canceled')

            # Guide only on first pick attempt (retries are automated Z correction).
            if use_guide_mode and pick_attempt == 0:
                self._feedback(goal_handle, 'GUIDE_MODE',
                               'Guide mode enabled — adjust grasp manually')
                for attempt in range(self._guide_mode_max_retries):
                    resp = self._svc.guide_mode('enable')
                    if resp.get('success'):
                        guide_was_enabled = True
                        break
                    self.get_logger().warn(
                        f'Guide mode enable attempt {attempt + 1} failed, retrying...')
                    time.sleep(self._guide_mode_retry_sleep_sec)
                time.sleep(self._guide_mode_settle_sec)

            if self._canceled(goal_handle):
                self._cleanup(guide_was_enabled)
                return self._result(False, 'CANCELED', 'Canceled')

            self._feedback(goal_handle, 'CLOSING_GRIPPER', 'Grasping object')
            self._wmc.gripper_close_if_needed(self._svc)
            time.sleep(self._sleep_after_close_gripper_sec)
            if self._grasp_verify_settle_sec > 0.0:
                time.sleep(self._grasp_verify_settle_sec)

            if self._canceled(goal_handle):
                self._cleanup(guide_was_enabled)
                return self._result(False, 'CANCELED', 'Canceled')

            if self._wmc.pick_grasp_verified(grasp_verify, max_open):
                break

            if pick_attempt >= max_pick_attempts - 1:
                self._cleanup(guide_was_enabled)
                goal_handle.abort()
                return self._result(
                    False, 'GRASP_VERIFY_FAILED',
                    'Grasp not verified after pick attempts with Z offsets')

        if use_guide_mode and guide_was_enabled:
            self._feedback(goal_handle, 'DISABLING_GUIDE_MODE')
            self._svc.guide_mode('disable')
            guide_was_enabled = False
            time.sleep(self._post_guide_settle_sec)

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 8. Retract to pre-pick (after verified grasp) ---
        self._feedback(goal_handle, 'RETRACTING', 'Retracting to pre-pick')
        pose_cmd = self._svc.compute_poses(
            {'query': 'pre_pick'}, timeout=self._compute_poses_timeout_sec)
        if pose_cmd.get('success'):
            pose_cmd['speed'] = pick_speed
            self._svc.move_cartesian_cmd(
                pose_cmd, timeout=self._move_timeout_sec)
        self._after_motion()

        self._svc.set_octomap_enabled(True)
        self._svc.clear_eef_bounds()

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        # --- 10. Retreat to approach ---
        self._feedback(goal_handle, 'RETREATING', 'Retreating to approach pose')
        pose_cmd = self._svc.compute_poses(
            {'query': 'approach_pick'}, timeout=self._compute_poses_timeout_sec)
        if pose_cmd.get('success'):
            self._svc.move_cartesian_cmd(
                pose_cmd, timeout=self._move_timeout_sec)
        self._after_motion()

        goal_handle.succeed()
        return self._result(True, '', 'Pick complete')


def main():
    rclpy.init()
    node = PickActionServer()
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
