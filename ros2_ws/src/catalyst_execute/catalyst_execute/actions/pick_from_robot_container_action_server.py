#!/usr/bin/env python3
"""Pick well plate from the robot-side container (hardcoded poses + guide grasp).

Assumes the arm starts at home.
Sequence:
  1. Optional settle
  2. Octomap ON → down_right → pre_pick (container hover) → open gripper
  3. Remove workspace_box collision object from planning scene (restore after retract)
  4. Octomap OFF → Cartesian descent to pick_pose
  5. Guide mode (optional) → close gripper → disable guide
  6. Retract to pre_pick → re-add workspace_box → octomap ON → down_right → joint home

Goal (JSON):
  use_guide_mode: bool (default true)

Usage:
  ros2 run catalyst_execute pick_from_robot_container_action_server
"""

import json
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String as StringMsg
from catalyst_interfaces.action import ExecuteTask
from catalyst_execute.actions.robot_container_utils import (
    pose_cfg_to_cartesian_dict,
    workspace_box_restore_from_cfg,
)
from catalyst_execute.utils.execute_config import section
from catalyst_execute.utils.service_clients import RobotServiceClients
from catalyst_execute.utils.world_model_cache import WorldModelCache


class PickFromRobotContainerActionServer(Node):
    def __init__(self):
        super().__init__('pick_from_robot_container_action_server')
        cfg = section('robot_container_pick_action_server')
        self._action_name = cfg.get('action_name', '/pick_from_robot_container')
        self._robot_phase_topic = cfg.get('robot_phase_topic', '/robot_phase')
        self._phase_pub_queue_size = int(cfg.get('phase_pub_queue_size', 10))
        self._start_settle_sec = float(cfg.get('start_settle_sec', 0.8))
        self._motion_settle_sec = float(cfg.get('motion_settle_sec', 0.5))
        self._joint_home_speed = float(cfg.get('joint_home_speed', 0.1))
        self._default_pick_speed = float(cfg.get('default_pick_speed', 0.01))
        self._move_timeout_sec = float(cfg.get('move_timeout_sec', 60.0))
        self._guide_mode_settle_sec = float(cfg.get('guide_mode_settle_sec', 2.0))
        self._post_guide_settle_sec = float(cfg.get('post_guide_settle_sec', 3.0))
        self._guide_mode_max_retries = int(cfg.get('guide_mode_max_retries', 5))
        self._guide_mode_retry_sleep_sec = float(
            cfg.get('guide_mode_retry_sleep_sec', 0.5))
        self._sleep_after_open_gripper_sec = float(
            cfg.get('sleep_after_open_gripper_sec', 0.5))
        self._sleep_after_close_gripper_sec = float(
            cfg.get('sleep_after_close_gripper_sec', 1.0))
        self._workspace_box_id = cfg.get(
            'workspace_collision_object_id', 'workspace_box')
        self._workspace_box_removed = False
        self._workspace_box_remove_verify_max_rounds = int(
            cfg.get('workspace_box_remove_verify_max_rounds', 8))
        self._workspace_box_remove_verify_pause_sec = float(
            cfg.get('workspace_box_remove_verify_pause_sec', 0.3))
        self._workspace_box_restore = workspace_box_restore_from_cfg(cfg)

        self._down_right = cfg.get('down_right', {})
        self._pre_pick = cfg.get('pre_pick', {})
        self._pick_pose = cfg.get('pick_pose', {})

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
            f'Pick-from-robot-container action server ready on {self._action_name}')

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Pick-from-robot-container cancel requested')
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
        self.get_logger().info(f'[PickFromRobot] {phase}: {message}')

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
            self.get_logger().warn('[PickFromRobot] Cancel detected')
            goal_handle.canceled()
            return True
        return False

    def _after_motion(self):
        if self._motion_settle_sec > 0.0:
            time.sleep(self._motion_settle_sec)

    def _remove_workspace_box_for_pick(self):
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

    def _cleanup(self, guide_was_enabled):
        self._publish_phase('RECOVERY')
        if guide_was_enabled:
            try:
                self._svc.guide_mode('disable')
                time.sleep(self._post_guide_settle_sec)
            except Exception:
                pass
        if self._workspace_box_removed:
            try:
                retract_cmd = pose_cfg_to_cartesian_dict(self._pre_pick)
                retract_cmd['speed'] = self._default_pick_speed
                self._svc.move_cartesian_cmd(
                    retract_cmd, timeout=self._move_timeout_sec)
            except Exception:
                pass
            self._restore_workspace_box_collision_if_removed()
        try:
            self._svc.set_octomap_enabled(True)
        except Exception:
            pass
        try:
            self._svc.move_cartesian_cmd(
                pose_cfg_to_cartesian_dict(self._down_right),
                timeout=self._move_timeout_sec)
        except Exception:
            pass
        self._publish_phase('IDLE')

    def _execute_cb(self, goal_handle):
        try:
            cmd = (json.loads(goal_handle.request.command)
                   if goal_handle.request.command else {})
        except json.JSONDecodeError as e:
            goal_handle.abort()
            return self._result(False, 'INVALID_COMMAND', str(e))

        use_guide_mode = cmd.get('use_guide_mode', True)
        pick_speed = cmd.get('pick_speed', self._default_pick_speed)

        needed = ['joint', 'cartesian', 'gripper', 'guide_mode', 'octomap',
                  'clear_octomap', 'scene']
        if not self._svc.wait_for_services(names=needed):
            goal_handle.abort()
            return self._result(False, 'SERVICES_UNAVAILABLE',
                                'Required services not available')

        try:
            return self._run(goal_handle, use_guide_mode, pick_speed)
        except Exception as e:
            self.get_logger().error(f'Pick-from-robot-container failed: {e}')
            self._cleanup(False)
            goal_handle.abort()
            return self._result(False, 'EXCEPTION', str(e))

    def _run(self, goal_handle, use_guide_mode, pick_speed):
        guide_was_enabled = False
        self._workspace_box_removed = False

        if self._start_settle_sec > 0.0:
            self._feedback(
                goal_handle, 'SETTLING',
                f'Waiting {self._start_settle_sec:.1f}s')
            time.sleep(self._start_settle_sec)

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        if not self._down_right or not self._pre_pick or not self._pick_pose:
            goal_handle.abort()
            return self._result(False, 'CONFIG_ERROR',
                                'down_right / pre_pick / pick_pose missing in YAML')

        self._svc.set_octomap_enabled(True)
        self._svc.clear_octomap()

        self._feedback(goal_handle, 'TRANSIT', 'Moving to down_right')
        move = self._svc.move_cartesian_cmd(
            pose_cfg_to_cartesian_dict(self._down_right),
            timeout=self._move_timeout_sec)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'TRANSIT_FAILED', move.get('message', ''))
        self._after_motion()

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(goal_handle, 'PRE_PICK', 'Moving to pre_pick container')
        move = self._svc.move_cartesian_cmd(
            pose_cfg_to_cartesian_dict(self._pre_pick),
            timeout=self._move_timeout_sec)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'PRE_PICK_FAILED', move.get('message', ''))
        self._after_motion()

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(goal_handle, 'OPENING_GRIPPER', 'Open gripper at pre_pick')
        self._wmc.gripper_open_if_needed(self._svc)
        time.sleep(self._sleep_after_open_gripper_sec)

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(
            goal_handle, 'WORKSPACE_BOX',
            f'Removing {self._workspace_box_id} from planning scene for pick')
        if not self._remove_workspace_box_for_pick():
            goal_handle.abort()
            return self._result(
                False, 'WORKSPACE_SCENE_REMOVE_FAILED',
                f'scene_command remove failed for {self._workspace_box_id}.')

        self._feedback(goal_handle, 'OCTOMAP', 'Disabling octomap for pick')
        self._svc.set_octomap_enabled(False)

        self._feedback(goal_handle, 'PICKING', 'Descending to pick pose')
        pick_cmd = pose_cfg_to_cartesian_dict(self._pick_pose)
        pick_cmd['speed'] = pick_speed
        move = self._svc.move_cartesian_cmd(
            pick_cmd, timeout=self._move_timeout_sec)
        if not move.get('success'):
            self._restore_workspace_box_collision_if_removed()
            self._cleanup(guide_was_enabled)
            goal_handle.abort()
            return self._result(False, 'PICK_DESCENT_FAILED', move.get('message', ''))
        self._after_motion()

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        if use_guide_mode:
            self._feedback(goal_handle, 'GUIDE_MODE', 'Guide mode — adjust grasp')
            for attempt in range(self._guide_mode_max_retries):
                resp = self._svc.guide_mode('enable')
                if resp.get('success'):
                    guide_was_enabled = True
                    break
                self.get_logger().warn(
                    f'Guide mode enable attempt {attempt + 1} failed')
                time.sleep(self._guide_mode_retry_sleep_sec)
            time.sleep(self._guide_mode_settle_sec)

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(goal_handle, 'CLOSING_GRIPPER', 'Grasping')
        self._wmc.gripper_close_if_needed(self._svc)
        time.sleep(self._sleep_after_close_gripper_sec)

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        if use_guide_mode and guide_was_enabled:
            self._feedback(goal_handle, 'DISABLING_GUIDE_MODE', '')
            self._svc.guide_mode('disable')
            guide_was_enabled = False
            time.sleep(self._post_guide_settle_sec)

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(goal_handle, 'RETRACTING', 'Retract to pre_pick container')
        retract_cmd = pose_cfg_to_cartesian_dict(self._pre_pick)
        retract_cmd['speed'] = pick_speed
        move = self._svc.move_cartesian_cmd(
            retract_cmd, timeout=self._move_timeout_sec)
        self._after_motion()
        if not move.get('success'):
            self._cleanup(guide_was_enabled)
            goal_handle.abort()
            return self._result(False, 'RETRACT_FAILED', move.get('message', ''))

        self._restore_workspace_box_collision_if_removed()

        self._svc.set_octomap_enabled(True)
        self._svc.clear_octomap()

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(goal_handle, 'TRANSIT', 'Moving to down_right (exit path)')
        move = self._svc.move_cartesian_cmd(
            pose_cfg_to_cartesian_dict(self._down_right),
            timeout=self._move_timeout_sec)
        self._after_motion()
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'EXIT_TRANSIT_FAILED', move.get('message', ''))

        if self._canceled(goal_handle):
            self._cleanup(guide_was_enabled)
            return self._result(False, 'CANCELED', 'Canceled')

        self._feedback(goal_handle, 'HOME', 'Joint home')
        move = self._svc.move_home(speed=self._joint_home_speed)
        if not move.get('success'):
            goal_handle.abort()
            return self._result(False, 'HOME_FAILED', move.get('message', ''))

        goal_handle.succeed()
        return self._result(True, '', 'Pick from robot container complete')


def main():
    rclpy.init()
    node = PickFromRobotContainerActionServer()
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
