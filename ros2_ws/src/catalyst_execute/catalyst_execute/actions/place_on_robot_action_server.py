#!/usr/bin/env python3
"""Place well plate into the robot-side container (hardcoded poses + SDK admittance).

Assumes the arm is already at home (e.g. BT joint home after /pick).
Sequence:
  1. Optional settle
  2. Transit: optional fixed joint path home → down_right → pre_place
     (``container_transit_joint_waypoints.enabled``), else Cartesian (octomap ON)
  3. At pre_place: remove workspace_box collision (restore after MoveIt exit; avoids ACM / GetPlanningScene)
  4. Octomap OFF
  5. SDK connect → simple_admittance (tool-frame VC + F/T) until seated or timeout
  6. Open gripper → SDK get_position + position_move +Z lift (mm) → sdk_control cleanup
     + restart_controllers (same cleanup path as before — not test_place_admittance)
  7. Octomap ON → re-add workspace_box → down_right (joint or Cartesian) → joint home

Goal (JSON): optional overrides, e.g. ref_velocity, descent_force_threshold,
admittance_max_distance, admittance_timeout_sec (see catalyst_execute_params.yaml).

Usage:
  ros2 run catalyst_execute place_on_robot_action_server
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
    parse_container_joint_transit,
    pose_cfg_to_cartesian_dict,
    sdk_z_delta_position_move_cmd,
    workspace_box_restore_from_cfg,
)
from catalyst_execute.utils.execute_config import section
from catalyst_execute.utils.service_clients import RobotServiceClients
from catalyst_execute.utils.world_model_cache import WorldModelCache


class PlaceOnRobotActionServer(Node):
    def __init__(self):
        super().__init__('place_on_robot_action_server')
        cfg = section('robot_container_place_action_server')
        self._action_name = cfg.get('action_name', '/place_on_robot')
        self._robot_phase_topic = cfg.get('robot_phase_topic', '/robot_phase')
        self._phase_pub_queue_size = int(cfg.get('phase_pub_queue_size', 10))
        self._start_settle_sec = float(cfg.get('start_settle_sec', 0.8))
        self._motion_settle_sec = float(cfg.get('motion_settle_sec', 0.5))
        self._joint_home_speed = float(cfg.get('joint_home_speed', 0.1))
        self._down_right = cfg.get('down_right', {})
        self._pre_place = cfg.get('pre_place', {})

        self._default_descent_force = float(
            cfg.get('default_descent_force_threshold', 5.0))
        self._default_ref_velocity = float(
            cfg.get('default_ref_velocity', -0.005))
        self._admittance_rate = float(cfg.get('admittance_rate', 100.0))
        self._admittance_linear_gain = float(
            cfg.get('admittance_linear_gain', 0.005))
        self._admittance_force_deadzone = float(
            cfg.get('admittance_force_deadzone', 1.0))
        self._admittance_max_linear_vel = float(
            cfg.get('admittance_max_linear_vel', 0.05))
        self._admittance_max_distance = float(
            cfg.get('admittance_max_distance', 0.1))
        self._admittance_compliant_axes = list(
            cfg.get('admittance_compliant_axes', [1, 1, 0, 0, 0, 0]))
        self._admittance_timeout_sec = float(
            cfg.get('admittance_timeout_sec', 120.0))
        self._sdk_cleanup_timeout_sec = float(
            cfg.get('sdk_cleanup_timeout_sec', 30.0))
        self._sdk_connect_timeout_sec = float(
            cfg.get('sdk_connect_timeout_sec', 30.0))
        self._restart_controllers_timeout_sec = float(
            cfg.get('restart_controllers_timeout_sec', 60.0))
        self._wait_moveit_services_timeout_sec = float(
            cfg.get('wait_moveit_services_timeout_sec', 30.0))
        self._move_cartesian_timeout_sec = float(
            cfg.get('move_cartesian_timeout_sec', 60.0))
        self._sleep_after_release_sec = float(
            cfg.get('sleep_after_release_sec', 0.5))
        self._sleep_after_restart_sec = float(
            cfg.get('sleep_after_restart_sec', 1.0))
        self._sdk_post_release_z_lift_mm = float(
            cfg.get('sdk_post_release_z_lift_mm', 50.0))
        self._sdk_pre_place_retract_speed_mm_s = float(
            cfg.get('sdk_pre_place_retract_speed_mm_s', 30.0))
        self._sdk_pre_place_retract_tolerance_mm = float(
            cfg.get('sdk_pre_place_retract_tolerance_mm', 1.0))
        self._sdk_pre_place_retract_timeout_sec = float(
            cfg.get('sdk_pre_place_retract_timeout_sec', 120.0))
        self._workspace_box_id = cfg.get(
            'workspace_collision_object_id', 'workspace_box')
        self._workspace_box_removed = False
        self._workspace_box_remove_verify_max_rounds = int(
            cfg.get('workspace_box_remove_verify_max_rounds', 8))
        self._workspace_box_remove_verify_pause_sec = float(
            cfg.get('workspace_box_remove_verify_pause_sec', 0.3))
        self._workspace_box_restore = workspace_box_restore_from_cfg(cfg)

        self._joint_transit = parse_container_joint_transit(cfg)
        _jt_raw = cfg.get('container_transit_joint_waypoints')
        if isinstance(_jt_raw, dict) and _jt_raw.get('enabled') and self._joint_transit is None:
            self.get_logger().warn(
                'container_transit_joint_waypoints.enabled but joints_down_right / '
                'joints_pre_container invalid — using Cartesian transit')

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
        self.get_logger().info(
            f'Place-on-robot action server ready on {self._action_name}')

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Place-on-robot cancel requested')
        return CancelResponse.ACCEPT

    def _publish_phase(self, phase):
        msg = StringMsg()
        msg.data = phase
        self._phase_pub.publish(msg)

    def _feedback(self, goal_handle, phase, message=''):
        self._publish_phase(phase)
        fb = ExecuteTask.Feedback()
        fb.feedback = json.dumps({'phase': phase, 'message': message})
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'[PlaceOnRobot] {phase}: {message}')

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
            self.get_logger().warn('[PlaceOnRobot] Cancel detected')
            goal_handle.canceled()
            return True
        return False

    def _after_motion(self):
        if self._motion_settle_sec > 0.0:
            time.sleep(self._motion_settle_sec)

    def _nominal_pre_place_cartesian_dict(self):
        """Container hover: YAML pre_place only (no planar offsets)."""
        return pose_cfg_to_cartesian_dict(self._pre_place)

    def _sdk_cleanup(self):
        self.get_logger().info('SDK cleanup...')
        self._svc.sdk_call(
            {'action': 'cleanup'}, timeout=self._sdk_cleanup_timeout_sec)

    def _remove_workspace_box_for_place(self):
        r = self._svc.ensure_collision_object_removed(
            self._workspace_box_id,
            max_rounds=self._workspace_box_remove_verify_max_rounds,
            pause_sec=self._workspace_box_remove_verify_pause_sec,
        )
        if r.get('success'):
            self._workspace_box_removed = True
            return True
        self.get_logger().warn(
            f'{self._workspace_box_id} removal not verified: '
            f'{r.get("message", "")}')
        return False

    def _restore_workspace_box_collision_if_removed(self):
        if not self._workspace_box_removed:
            return
        g = self._workspace_box_restore
        r = self._svc.add_collision_box(
            self._workspace_box_id,
            g['position'],
            g['dimensions'],
            orientation=g['orientation'],
            frame_id=g['frame_id'],
        )
        if r.get('success'):
            self._workspace_box_removed = False
        else:
            self.get_logger().error(
                f'Failed to restore {self._workspace_box_id} collision: '
                f'{r.get("message", "")}')

    def _recover_retract_home(self, goal_handle):
        """Best-effort: restart MoveIt path, octomap on, workspace collisions, down_right, home."""
        self._feedback(goal_handle, 'RECOVERY', 'Recovering after failure')
        try:
            self._sdk_cleanup()
        except Exception:
            pass
        try:
            resp = self._svc.sdk_call(
                {'action': 'restart_controllers'},
                timeout=self._restart_controllers_timeout_sec)
            if resp.get('success'):
                self._svc.wait_for_services(
                    names=['joint', 'cartesian'],
                    timeout=self._wait_moveit_services_timeout_sec)
                time.sleep(self._sleep_after_restart_sec)
        except Exception:
            pass
        try:
            self._svc.set_octomap_enabled(True)
        except Exception:
            pass
        self._restore_workspace_box_collision_if_removed()
        try:
            if self._joint_transit:
                self._svc.move_joints_deg(
                    self._joint_transit['joints_down_right'],
                    speed=self._joint_transit['speed'],
                    timeout=self._move_cartesian_timeout_sec)
            else:
                dr = pose_cfg_to_cartesian_dict(self._down_right)
                self._svc.move_cartesian_cmd(
                    dr, timeout=self._move_cartesian_timeout_sec)
        except Exception:
            pass
        try:
            self._svc.move_home(speed=self._joint_home_speed)
        except Exception:
            pass

    def _execute_cb(self, goal_handle):
        try:
            cmd = (json.loads(goal_handle.request.command)
                   if goal_handle.request.command else {})
        except json.JSONDecodeError as e:
            goal_handle.abort()
            return self._result(False, 'INVALID_COMMAND', str(e))

        needed = [
            'joint', 'cartesian', 'gripper', 'sdk',
            'octomap', 'clear_octomap', 'scene',
        ]
        if not self._svc.wait_for_services(names=needed):
            goal_handle.abort()
            return self._result(False, 'SERVICES_UNAVAILABLE',
                                'Required services not available')

        try:
            return self._run(goal_handle, cmd)
        except Exception as e:
            self.get_logger().error(f'Place-on-robot failed: {e}')
            self._restore_workspace_box_collision_if_removed()
            self._sdk_cleanup()
            goal_handle.abort()
            return self._result(False, 'EXCEPTION', str(e))

    def _run(self, goal_handle, goal_cmd):

        self._workspace_box_removed = False

        if self._start_settle_sec > 0.0:
            self._feedback(
                goal_handle, 'SETTLING',
                f'Waiting {self._start_settle_sec:.1f}s')
            time.sleep(self._start_settle_sec)

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        if not self._down_right or not self._pre_place:
            goal_handle.abort()
            return self._result(False, 'CONFIG_ERROR',
                                'down_right / pre_place missing in YAML')

        # Approach with octomap ON (planning sees obstacles)
        self._svc.set_octomap_enabled(True)
        self._svc.clear_octomap()

        to = self._move_cartesian_timeout_sec
        jt = self._joint_transit
        if jt:
            self._feedback(goal_handle, 'TRANSIT', 'Joint path: home')
            move = self._svc.move_home(speed=self._joint_home_speed, timeout=to)
            if not move.get('success'):
                goal_handle.abort()
                return self._result(False, 'TRANSIT_FAILED', move.get('message', ''))
            self._after_motion()
            if self._canceled(goal_handle):
                return self._result(False, 'CANCELED', 'Canceled')
            self._feedback(goal_handle, 'TRANSIT', 'Joint path: down_right')
            move = self._svc.move_joints_deg(
                jt['joints_down_right'], speed=jt['speed'], timeout=to)
            if not move.get('success'):
                goal_handle.abort()
                return self._result(False, 'TRANSIT_FAILED', move.get('message', ''))
            self._after_motion()
            if self._canceled(goal_handle):
                return self._result(False, 'CANCELED', 'Canceled')
            self._feedback(
                goal_handle, 'PRE_PLACE',
                'Joint path: pre_place container (nominal)')
            move = self._svc.move_joints_deg(
                jt['joints_pre_container'], speed=jt['speed'], timeout=to)
            if not move.get('success'):
                goal_handle.abort()
                return self._result(False, 'PRE_PLACE_FAILED', move.get('message', ''))
            self._after_motion()
        else:
            self._feedback(goal_handle, 'TRANSIT', 'Moving to down_right')
            move = self._svc.move_cartesian_cmd(
                pose_cfg_to_cartesian_dict(self._down_right),
                timeout=to)
            if not move.get('success'):
                goal_handle.abort()
                return self._result(False, 'TRANSIT_FAILED', move.get('message', ''))
            self._after_motion()

            if self._canceled(goal_handle):
                return self._result(False, 'CANCELED', 'Canceled')

            self._feedback(
                goal_handle, 'PRE_PLACE',
                'Moving to pre_place container (nominal)')
            move = self._svc.move_cartesian_cmd(
                self._nominal_pre_place_cartesian_dict(),
                timeout=to)
            if not move.get('success'):
                goal_handle.abort()
                return self._result(False, 'PRE_PLACE_FAILED', move.get('message', ''))
            self._after_motion()

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(
            goal_handle, 'WORKSPACE_BOX',
            'Removing workspace_box collision for gripper clearance')
        if not self._remove_workspace_box_for_place():
            goal_handle.abort()
            return self._result(
                False, 'WORKSPACE_SCENE_REMOVE_FAILED',
                'scene_command failed — workspace_box not removed from planning scene.')

        self._feedback(goal_handle, 'OCTOMAP', 'Disabling octomap for container place')
        self._svc.set_octomap_enabled(False)

        # --- SDK: connect + admittance insert (simple_admittance in sdk_control_service) ---
        self._feedback(goal_handle, 'SDK_CONNECTING', 'SDK connect')
        resp = self._svc.sdk_call(
            {'action': 'connect'}, timeout=self._sdk_connect_timeout_sec)
        if not resp.get('success'):
            try:
                self._svc.set_octomap_enabled(True)
            except Exception:
                pass
            self._restore_workspace_box_collision_if_removed()
            goal_handle.abort()
            return self._result(False, 'SDK_CONNECT_FAILED',
                                resp.get('message', ''))

        if self._canceled(goal_handle):
            self._sdk_cleanup()
            self._recover_retract_home(goal_handle)
            return self._result(False, 'CANCELED', 'Canceled')

        gc = goal_cmd
        compliant = gc.get('admittance_compliant_axes', self._admittance_compliant_axes)
        admittance_vc = {
            'action': 'velocity_control',
            'mode': 'simple_admittance',
            'rate': float(gc.get('admittance_rate', self._admittance_rate)),
            'ref_velocity': float(
                gc.get('ref_velocity', self._default_ref_velocity)),
            'force_threshold': float(
                gc.get('descent_force_threshold', self._default_descent_force)),
            'max_distance': float(
                gc.get('admittance_max_distance', self._admittance_max_distance)),
            'linear_gain': float(
                gc.get('admittance_linear_gain', self._admittance_linear_gain)),
            'force_deadzone': float(
                gc.get('admittance_force_deadzone', self._admittance_force_deadzone)),
            'compliant_axes': compliant,
            'max_linear_vel': float(
                gc.get('admittance_max_linear_vel', self._admittance_max_linear_vel)),
            'timeout_sec': float(
                gc.get('admittance_timeout_sec', self._admittance_timeout_sec)),
        }
        admittance_call_timeout = max(
            180.0, admittance_vc['timeout_sec'] + 60.0)

        self._feedback(goal_handle, 'ADMITTANCE', 'Admittance placement (tool VC + F/T)')
        resp = self._svc.sdk_call(admittance_vc, timeout=admittance_call_timeout)
        self._feedback(goal_handle, 'ADMITTANCE', resp.get('message', ''))
        self._after_motion()

        if not resp.get('success'):
            self._sdk_cleanup()
            self._recover_retract_home(goal_handle)
            goal_handle.abort()
            return self._result(False, 'ADMITTANCE_FAILED',
                                resp.get('message', ''))

        if self._canceled(goal_handle):
            self._sdk_cleanup()
            self._recover_retract_home(goal_handle)
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(goal_handle, 'RELEASING', 'Opening gripper')
        self._wmc.gripper_open_if_needed(self._svc)
        time.sleep(self._sleep_after_release_sec)

        self._feedback(
            goal_handle, 'SDK_RETRACT',
            f'SDK position move +Z {self._sdk_post_release_z_lift_mm:.0f} mm (clear container)')
        pos_resp = self._svc.sdk_call({'action': 'get_position'})
        if not pos_resp.get('success'):
            self.get_logger().warn(
                f'SDK get_position before Z lift: {pos_resp.get("message", "")}')
        else:
            z_lift_cmd = sdk_z_delta_position_move_cmd(
                pos_resp['position'],
                self._sdk_post_release_z_lift_mm,
                speed_mm_s=self._sdk_pre_place_retract_speed_mm_s,
                tolerance_mm=self._sdk_pre_place_retract_tolerance_mm,
            )
            move = self._svc.sdk_call(
                z_lift_cmd,
                timeout=self._sdk_pre_place_retract_timeout_sec)
            if not move.get('success'):
                self.get_logger().warn(
                    f'SDK Z lift retract: {move.get("message", "")}')

        self._after_motion()

        self._feedback(goal_handle, 'CLEANUP', 'SDK cleanup + restart controllers')
        self._sdk_cleanup()

        resp = self._svc.sdk_call(
            {'action': 'restart_controllers'},
            timeout=self._restart_controllers_timeout_sec)
        if not resp.get('success'):
            self._recover_retract_home(goal_handle)
            goal_handle.abort()
            return self._result(False, 'CONTROLLER_RESTART_FAILED',
                                resp.get('message', ''))

        self._feedback(goal_handle, 'WAITING_FOR_SERVICES', 'Waiting for MoveIt')
        self._svc.wait_for_services(
            names=['joint', 'cartesian'],
            timeout=self._wait_moveit_services_timeout_sec)
        time.sleep(self._sleep_after_restart_sec)

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(goal_handle, 'OCTOMAP', 'Re-enabling octomap after retract')
        self._svc.set_octomap_enabled(True)
        self._svc.clear_octomap()

        self._restore_workspace_box_collision_if_removed()

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(goal_handle, 'TRANSIT', 'Exit: down_right')
        to = self._move_cartesian_timeout_sec
        if self._joint_transit:
            move = self._svc.move_joints_deg(
                self._joint_transit['joints_down_right'],
                speed=self._joint_transit['speed'],
                timeout=to)
        else:
            move = self._svc.move_cartesian_cmd(
                pose_cfg_to_cartesian_dict(self._down_right),
                timeout=to)
        self._after_motion()
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'EXIT_TRANSIT_FAILED', move.get('message', ''))

        if self._canceled(goal_handle):
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(goal_handle, 'HOME', 'Joint home')
        move = self._svc.move_home(speed=self._joint_home_speed)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'HOME_FAILED', move.get('message', ''))

        goal_handle.succeed()
        return self._result(True, '', 'Place on robot complete')


def main():
    rclpy.init()
    node = PlaceOnRobotActionServer()
    n_threads = max(8, (os.cpu_count() or 4) * 2)
    executor = MultiThreadedExecutor(num_threads=n_threads)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._sdk_cleanup()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
