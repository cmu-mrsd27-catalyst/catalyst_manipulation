#!/usr/bin/env python3
"""Well-plate detection action server (ExecuteTask).

Moves the TCP to a viewpoint calibrated from tag frame (same idea as pick/place),
captures a camera frame, POSTs it to the remote GPU detection server, then (unless
``skip_motion``) moves the arm to **joint home**, then to **approach_pick_pre** when
``require`` is ``pick_safe`` and the server reports **pick_safe**. There is no
post-detect approach-place motion (arm stays at home for container-pick then place).
**Success** follows ``require``: ``pick_safe`` gates on **pick_safe** only;
``place_safe`` gates on **place_safe** only. Returns pick/place safety flags from the server.

Goal (JSON):
  tag_pose: required unless skip_motion is true — {position: {x,y,z},
            orientation: {qx,qy,qz,qw}} in link_base (e.g. merged from explore_result)
  skip_motion: optional bool — if true, skip cartesian move (use current camera view)
  require: optional string — omit | \"pick_safe\" | \"place_safe\"
            Gates success on that flag only (no combined pick+place check).
            If set, success is false when the corresponding server flag is false
            (action still completes HTTP; BT sees success: false).

Result (JSON):
  success: bool — motion + HTTP ok and passes \"require\" checks
  error_code: string — e.g. MOTION_FAILED, POST_DETECT_HOME_FAILED,
            POST_DETECT_POSE_FAILED, POST_DETECT_APPROACH_FAILED,
            DETECTION_HTTP_FAILED, PICK_UNSAFE
  message: string
  pick_safe, place_safe: bool (from server pick_place)
  detection: optional object — full server JSON when inference succeeded

Usage:
  ros2 run catalyst_execute detect_well_plate_action_server
"""

import json
import os
import time
from io import BytesIO

import cv2
import numpy as np
import requests
import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String as StringMsg

from catalyst_interfaces.action import ExecuteTask
from catalyst_execute.utils.execute_config import section
from catalyst_execute.utils.pose_math import (
    homogeneous_to_pos_quat,
    pose_to_homogeneous,
)
from catalyst_execute.utils.service_clients import RobotServiceClients


class DetectWellPlateActionServer(Node):
    def __init__(self):
        super().__init__('detect_well_plate_action_server')
        cfg = section('detect_well_plate_action_server')
        self._action_name = cfg.get('action_name', '/detect_well_plate')
        self._robot_phase_topic = cfg.get('robot_phase_topic', '/robot_phase')
        self._phase_pub_queue_size = int(cfg.get('phase_pub_queue_size', 10))
        self._camera_topic = cfg.get(
            'camera_topic', '/camera/camera/color/image_raw')
        self._server_url = str(cfg.get('server_url', 'http://127.0.0.1:8000')).rstrip('/')
        self._http_timeout_sec = float(cfg.get('http_timeout_sec', 60.0))
        self._move_cartesian_timeout_sec = float(
            cfg.get('move_cartesian_timeout_sec', 60.0))
        self._start_settle_sec = float(cfg.get('start_settle_sec', 0.8))
        self._motion_settle_sec = float(cfg.get('motion_settle_sec', 0.5))
        self._fresh_image_wait_sec = float(cfg.get('fresh_image_wait_sec', 3.0))
        self._default_move_speed = float(cfg.get('default_move_speed', 0.1))
        self._jpeg_quality = int(cfg.get('jpeg_quality', 90))
        self._post_detect_home_speed = float(cfg.get('post_detect_home_speed', 0.3))
        self._joint_move_timeout_sec = float(cfg.get('joint_move_timeout_sec', 60.0))
        self._compute_poses_timeout_sec = float(
            cfg.get('compute_poses_timeout_sec', 10.0))

        tcp_cal = cfg.get('calibration_tcp_pose')
        tag_cal = cfg.get('calibration_tag_pose_in_base')
        if not tcp_cal or not tag_cal:
            self.get_logger().error(
                'detect_well_plate_action_server: calibration_tcp_pose and '
                'calibration_tag_pose_in_base are required in YAML')
            self._H_tag_to_tcp = None
        else:
            H_tcp = pose_to_homogeneous(tcp_cal['position'], tcp_cal['orientation'])
            H_tag = pose_to_homogeneous(tag_cal['position'], tag_cal['orientation'])
            self._H_tag_to_tcp = np.linalg.inv(H_tag) @ H_tcp

        self._cb_group = ReentrantCallbackGroup()
        self._svc = RobotServiceClients(self, self._cb_group)
        self._bridge = CvBridge()
        self._latest_image = None
        self._image_arrival_ns = 0

        self.create_subscription(
            Image,
            self._camera_topic,
            self._image_cb,
            1,
            callback_group=self._cb_group,
        )

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
        self.get_logger().info(
            f'Detect well plate action ready on {self._action_name} '
            f'(server {self._server_url}, camera {self._camera_topic})')

    def _image_cb(self, msg: Image):
        self._latest_image = msg
        self._image_arrival_ns = self.get_clock().now().nanoseconds

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Detect well plate cancel requested')
        return CancelResponse.ACCEPT

    def _publish_phase(self, phase):
        m = StringMsg()
        m.data = phase
        self._phase_pub.publish(m)

    def _feedback(self, goal_handle, phase, message=''):
        self._publish_phase(phase)
        fb = ExecuteTask.Feedback()
        fb.feedback = json.dumps({'phase': phase, 'message': message})
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'[DetectWellPlate] {phase}: {message}')

    def _tcp_pose_from_tag(self, tag_pose: dict):
        if self._H_tag_to_tcp is None:
            return None
        H_tag = pose_to_homogeneous(tag_pose['position'], tag_pose['orientation'])
        H_tcp = H_tag @ self._H_tag_to_tcp
        return homogeneous_to_pos_quat(H_tcp)

    def _cartesian_cmd_from_pose(self, pose7, speed):
        x, y, z, qx, qy, qz, qw = pose7
        return {
            'success': True,
            'message': 'OK',
            'x': float(x),
            'y': float(y),
            'z': float(z),
            'qx': float(qx),
            'qy': float(qy),
            'qz': float(qz),
            'qw': float(qw),
            'speed': float(speed),
            'keep_orientation': False,
            'straight_line': False,
        }

    def _wait_fresh_image(self, after_ns: int, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if (self._latest_image is not None
                    and self._image_arrival_ns > after_ns):
                return True
            time.sleep(0.02)
        return False

    def _run_http_detect(self):
        if self._latest_image is None:
            return None, 'No camera image available'
        try:
            cv_image = self._bridge.imgmsg_to_cv2(
                self._latest_image, desired_encoding='bgr8')
        except Exception as e:
            return None, f'cv_bridge failed: {e}'

        ok, buffer = cv2.imencode(
            '.jpg', cv_image,
            [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
        )
        if not ok:
            return None, 'Failed to encode JPEG'

        try:
            resp = requests.post(
                f'{self._server_url}/detect',
                files={'file': ('image.jpg', BytesIO(buffer.tobytes()), 'image/jpeg')},
                timeout=self._http_timeout_sec,
            )
        except requests.exceptions.ConnectionError:
            return None, f'Cannot connect to {self._server_url}'
        except requests.exceptions.Timeout:
            return None, f'Detection server timed out ({self._http_timeout_sec}s)'

        if resp.status_code != 200:
            return None, f'Server HTTP {resp.status_code}: {resp.text[:200]}'

        try:
            return resp.json(), ''
        except json.JSONDecodeError as e:
            return None, f'Invalid JSON from server: {e}'

    def _execute_cb(self, goal_handle):
        try:
            cmd = json.loads(goal_handle.request.command)
        except json.JSONDecodeError as e:
            goal_handle.abort()
            r = ExecuteTask.Result()
            r.response = json.dumps({
                'success': False,
                'error_code': 'BAD_GOAL',
                'message': str(e),
            })
            return r

        skip_motion = bool(cmd.get('skip_motion', False))
        require = (cmd.get('require', '') or '').strip()
        if require not in ('', 'pick_safe', 'place_safe'):
            goal_handle.abort()
            r = ExecuteTask.Result()
            r.response = json.dumps({
                'success': False,
                'error_code': 'BAD_GOAL',
                'message': (
                    f'require must be omit, pick_safe, or place_safe (got {require!r})'
                ),
            })
            return r
        tag_pose = cmd.get('tag_pose')
        move_speed = float(cmd.get('speed', self._default_move_speed))

        if not skip_motion:
            if not tag_pose or 'position' not in tag_pose or 'orientation' not in tag_pose:
                goal_handle.abort()
                r = ExecuteTask.Result()
                r.response = json.dumps({
                    'success': False,
                    'error_code': 'MISSING_TAG_POSE',
                    'message': 'tag_pose required unless skip_motion is true',
                })
                return r
            if self._H_tag_to_tcp is None:
                goal_handle.abort()
                r = ExecuteTask.Result()
                r.response = json.dumps({
                    'success': False,
                    'error_code': 'NO_CALIBRATION',
                    'message': 'YAML calibration_tcp_pose / calibration_tag_pose_in_base missing',
                })
                return r

        needed_svcs = ['cartesian']
        if not skip_motion:
            needed_svcs = ['cartesian', 'joint', 'octomap']
            if require == 'pick_safe':
                needed_svcs.append('compute_poses')
        if not self._svc.wait_for_services(names=needed_svcs, timeout=30.0):
            goal_handle.abort()
            r = ExecuteTask.Result()
            r.response = json.dumps({
                'success': False,
                'error_code': 'SERVICES_UNAVAILABLE',
                'message': f'Required services not available: {needed_svcs}',
            })
            return r

        if goal_handle.is_cancel_requested:
            return self._canceled_result(goal_handle)

        if self._start_settle_sec > 0.0:
            self._feedback(
                goal_handle, 'SETTLING',
                f'Waiting {self._start_settle_sec:.1f}s before motion')
            time.sleep(self._start_settle_sec)

        if not skip_motion:
            self._feedback(goal_handle, 'MOVING', 'Cartesian move to detection pose')
            pose7 = self._tcp_pose_from_tag(tag_pose)
            pose_cmd = self._cartesian_cmd_from_pose(pose7, move_speed)
            self._svc.set_octomap_enabled(False)
            move = self._svc.move_cartesian_cmd(
                pose_cmd, timeout=self._move_cartesian_timeout_sec)
            if not move.get('success'):
                self._publish_phase('IDLE')
                goal_handle.abort()
                r = ExecuteTask.Result()
                r.response = json.dumps({
                    'success': False,
                    'error_code': 'MOTION_FAILED',
                    'message': move.get('message', 'cartesian failed'),
                })
                return r

        if self._motion_settle_sec > 0.0:
            time.sleep(self._motion_settle_sec)

        threshold_ns = self.get_clock().now().nanoseconds
        self._feedback(goal_handle, 'CAPTURE', 'Waiting for fresh camera frame')
        if not self._wait_fresh_image(threshold_ns, self._fresh_image_wait_sec):
            self._publish_phase('IDLE')
            goal_handle.abort()
            r = ExecuteTask.Result()
            r.response = json.dumps({
                'success': False,
                'error_code': 'NO_FRESH_IMAGE',
                'message': f'No image newer than pose settle within {self._fresh_image_wait_sec}s',
            })
            return r

        if goal_handle.is_cancel_requested:
            return self._canceled_result(goal_handle)

        self._feedback(goal_handle, 'DETECTING', f'POST {self._server_url}/detect')
        det, err = self._run_http_detect()
        self._publish_phase('IDLE')

        if det is None:
            goal_handle.abort()
            r = ExecuteTask.Result()
            r.response = json.dumps({
                'success': False,
                'error_code': 'DETECTION_HTTP_FAILED',
                'message': err,
            })
            return r

        pp = det.get('pick_place') or {}
        pick_safe = bool(pp.get('pick_safe', False))
        place_safe = bool(pp.get('place_safe', False))

        ok_require = True
        err_code = ''
        if require == 'pick_safe':
            ok_require = pick_safe
            err_code = 'PICK_UNSAFE' if not pick_safe else ''
        elif require == 'place_safe':
            ok_require = place_safe
            err_code = 'PLACE_UNSAFE' if not place_safe else ''

        if not skip_motion:
            self._feedback(goal_handle, 'POST_DETECT_HOME', 'Joint home after detection')
            hm = self._svc.move_home(
                speed=self._post_detect_home_speed,
                timeout=self._joint_move_timeout_sec,
            )
            if not hm.get('success'):
                self._publish_phase('IDLE')
                goal_handle.abort()
                r = ExecuteTask.Result()
                r.response = json.dumps({
                    'success': False,
                    'error_code': 'POST_DETECT_HOME_FAILED',
                    'message': hm.get('message', 'joint home failed'),
                    'pick_safe': pick_safe,
                    'place_safe': place_safe,
                    'detection': det,
                })
                return r

            if self._motion_settle_sec > 0.0:
                time.sleep(self._motion_settle_sec)

            do_pick_approach = require == 'pick_safe' and pick_safe

            if do_pick_approach:
                self._svc.set_octomap_enabled(False)
                cr = self._svc.compute_poses(
                    {'tag_pose': tag_pose},
                    timeout=self._compute_poses_timeout_sec,
                )
                if not cr.get('success'):
                    self._publish_phase('IDLE')
                    goal_handle.abort()
                    r = ExecuteTask.Result()
                    r.response = json.dumps({
                        'success': False,
                        'error_code': 'POST_DETECT_POSE_FAILED',
                        'message': cr.get('message', 'compute_poses tag_pose failed'),
                        'pick_safe': pick_safe,
                        'place_safe': place_safe,
                        'detection': det,
                    })
                    return r

                self._feedback(
                    goal_handle, 'POST_DETECT_APPROACH',
                    'Cartesian to approach_pick_pre after home',
                )
                pose_cmd = self._svc.compute_poses(
                    {'query': 'approach_pick_pre'},
                    timeout=self._compute_poses_timeout_sec,
                )
                if not pose_cmd.get('success'):
                    self._publish_phase('IDLE')
                    goal_handle.abort()
                    r = ExecuteTask.Result()
                    r.response = json.dumps({
                        'success': False,
                        'error_code': 'POST_DETECT_POSE_FAILED',
                        'message': pose_cmd.get('message', 'compute_poses query failed'),
                        'pick_safe': pick_safe,
                        'place_safe': place_safe,
                        'detection': det,
                    })
                    return r

                pose_cmd['speed'] = float(
                    cmd.get('post_detect_approach_speed', self._default_move_speed))
                mv = self._svc.move_cartesian_cmd(
                    pose_cmd, timeout=self._move_cartesian_timeout_sec)
                if not mv.get('success'):
                    self._publish_phase('IDLE')
                    goal_handle.abort()
                    r = ExecuteTask.Result()
                    r.response = json.dumps({
                        'success': False,
                        'error_code': 'POST_DETECT_APPROACH_FAILED',
                        'message': mv.get('message', 'approach cartesian failed'),
                        'pick_safe': pick_safe,
                        'place_safe': place_safe,
                        'detection': det,
                    })
                    return r

                if self._motion_settle_sec > 0.0:
                    time.sleep(self._motion_settle_sec)

        msg = (
            f'pick_safe={pick_safe} place_safe={place_safe} '
            f'inference_ms={det.get("inference_time_ms", "?")}'
        )
        body = {
            'success': ok_require,
            'error_code': err_code if not ok_require else '',
            'message': msg,
            'pick_safe': pick_safe,
            'place_safe': place_safe,
            'detection': det,
        }

        goal_handle.succeed()
        r = ExecuteTask.Result()
        r.response = json.dumps(body)
        return r

    def _canceled_result(self, goal_handle):
        self.get_logger().warn('[DetectWellPlate] Cancel detected')
        self._publish_phase('IDLE')
        goal_handle.canceled()
        r = ExecuteTask.Result()
        r.response = json.dumps({
            'success': False,
            'error_code': 'CANCELED',
            'message': 'Canceled',
        })
        return r


def main():
    rclpy.init()
    node = DetectWellPlateActionServer()
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
