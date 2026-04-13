"""Shared service client utilities for action servers.

Provides synchronous access to all robot JsonCommand / SetBool / Empty services.

Must be used from a node running inside a MultiThreadedExecutor so that
service response callbacks can be processed while the caller blocks.
"""

import json
import threading
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool

from catalyst_interfaces.srv import JsonCommand
from catalyst_execute.utils.execute_config import section


def _wait_ros_client_future(future, timeout_sec):
    """Block until *future* completes without a busy spin loop.

    Uses threading.Event + add_done_callback so the waiter yields until the
    executor delivers the response (works with MultiThreadedExecutor on the
    same node).
    """
    if future.done():
        return True
    done = threading.Event()

    def _on_done(_fut):
        done.set()

    future.add_done_callback(_on_done)
    if future.done():
        return True
    return done.wait(timeout=timeout_sec)


class RobotServiceClients:
    """Synchronous wrappers around all robot JsonCommand / SetBool / Empty services."""

    def __init__(self, node: Node, callback_group: ReentrantCallbackGroup):
        self._node = node
        self._log = node.get_logger()
        sc = section('service_clients')
        self._wait_for_services_timeout = float(
            sc.get('wait_for_services_timeout_sec', 30.0))
        self._scene_command_timeout = float(
            sc.get('scene_command_timeout_sec', 60.0))
        self._scene_command_max_retries = int(
            sc.get('scene_command_max_retries', 4))

        self._joint = node.create_client(
            JsonCommand, sc.get('joint_command', '/joint_command'),
            callback_group=callback_group)
        self._cartesian = node.create_client(
            JsonCommand, sc.get('cartesian_command', '/cartesian_command'),
            callback_group=callback_group)
        self._gripper = node.create_client(
            JsonCommand, sc.get('gripper_command', '/gripper_command'),
            callback_group=callback_group)
        self._guide_mode = node.create_client(
            JsonCommand, sc.get('guide_mode', '/guide_mode'),
            callback_group=callback_group)
        self._sdk = node.create_client(
            JsonCommand, sc.get('sdk_control', '/sdk_control'),
            callback_group=callback_group)
        self._compute_poses = node.create_client(
            JsonCommand, sc.get('compute_poses', '/compute_poses'),
            callback_group=callback_group)
        self._set_octomap = node.create_client(
            SetBool, sc.get('set_octomap_enabled', '/set_octomap_enabled'),
            callback_group=callback_group)
        self._clear_octomap = node.create_client(
            Empty, sc.get('clear_octomap', '/clear_octomap'),
            callback_group=callback_group)
        self._collision_sensitivity = node.create_client(
            JsonCommand,
            sc.get('set_collision_sensitivity', '/set_collision_sensitivity'),
            callback_group=callback_group)
        self._eef_bounds = node.create_client(
            JsonCommand, sc.get('set_eef_bounds', '/set_eef_bounds'),
            callback_group=callback_group)
        self._scene = node.create_client(
            JsonCommand, sc.get('scene_command', '/scene_command'),
            callback_group=callback_group)

    def wait_for_services(self, names=None, timeout=None):
        """Wait for required services.  *names* filters by key."""
        if timeout is None:
            timeout = self._wait_for_services_timeout
        all_clients = {
            'joint': self._joint,
            'cartesian': self._cartesian,
            'gripper': self._gripper,
            'guide_mode': self._guide_mode,
            'sdk': self._sdk,
            'compute_poses': self._compute_poses,
            'octomap': self._set_octomap,
            'clear_octomap': self._clear_octomap,
            'scene': self._scene,
        }
        clients = (
            {k: v for k, v in all_clients.items() if k in names}
            if names else all_clients
        )
        for name, client in clients.items():
            self._log.info(f'Waiting for {client.srv_name}...')
            if not client.wait_for_service(timeout_sec=timeout):
                self._log.error(f'{client.srv_name} not available')
                return False
        self._log.info('All required services ready.')
        return True

    def call_json(self, client, command, timeout=60.0):
        """Call a JsonCommand service synchronously.  Returns parsed dict."""
        req = JsonCommand.Request()
        req.command = json.dumps(command) if isinstance(command, dict) else command
        self._log.info(f'-> {client.srv_name}: {req.command[:200]}')

        future = client.call_async(req)
        if not _wait_ros_client_future(future, timeout):
            self._log.error(f'Timeout calling {client.srv_name}')
            return {'success': False, 'message': 'Service call timeout'}

        if future.result() is None:
            return {'success': False, 'message': 'Service call returned None'}

        resp = json.loads(future.result().response)
        self._log.info(f'<- {client.srv_name}: {resp.get("message", "")}')
        return resp

    def move_home(self, speed=0.1, timeout=60.0):
        return self.call_json(
            self._joint, {'pose': 'home', 'speed': speed}, timeout=timeout)

    def move_joints_deg(self, joints_deg, speed=0.1, timeout=60.0):
        """Joint-space move via /joint_command (six angles in degrees)."""
        return self.call_json(
            self._joint,
            {'joints': [float(x) for x in joints_deg], 'speed': float(speed)},
            timeout=timeout,
        )

    def move_cartesian(self, x, y, z, qx, qy, qz, qw, speed=0.1,
                       keep_orientation=False, straight_line=False):
        return self.call_json(self._cartesian, {
            'x': x, 'y': y, 'z': z,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'speed': speed,
            'keep_orientation': keep_orientation,
            'straight_line': straight_line,
        })

    def move_cartesian_cmd(self, cmd_dict, timeout=60.0):
        """Send a raw pose command dict to /cartesian_command."""
        return self.call_json(self._cartesian, cmd_dict, timeout=timeout)

    def gripper(self, action):
        return self.call_json(self._gripper, {'action': action})

    def guide_mode(self, action, sensitivity=5):
        return self.call_json(self._guide_mode,
                              {'action': action, 'sensitivity': sensitivity})

    def compute_poses(self, command, timeout=10.0):
        return self.call_json(self._compute_poses, command, timeout=timeout)

    def sdk_call(self, command, timeout=120.0):
        return self.call_json(self._sdk, command, timeout=timeout)

    def set_octomap_enabled(self, enabled):
        req = SetBool.Request()
        req.data = enabled
        future = self._set_octomap.call_async(req)
        if not _wait_ros_client_future(future, 10.0):
            self._log.error('Timeout setting octomap')
            return False
        result = future.result()
        if result is None:
            self._log.error('set_octomap_enabled: service returned None')
            return False
        if not result.success:
            self._log.warn(
                f'set_octomap_enabled({enabled}) rejected: {result.message}')
            return False
        return True

    def set_collision_sensitivity(self, level):
        """Set xArm collision sensitivity (0=off, 5=max)."""
        return self.call_json(self._collision_sensitivity, {'level': level}, timeout=10.0)

    def set_eef_bounds(self, tag_pose):
        """Compute and set EEF bounds from AprilTag pose."""
        tx = tag_pose['position']['x']
        tz = tag_pose['position']['z']
        return self.call_json(self._eef_bounds, {
            'action': 'set',
            'x_min': tx - 0.085,
            'x_max': tx + 0.135,
            'z_min': tz + 0.141,
            'z_max': tz + 0.381,
        }, timeout=10.0)

    def clear_eef_bounds(self):
        """Clear EEF position bounds."""
        return self.call_json(self._eef_bounds, {'action': 'clear'}, timeout=10.0)

    def clear_octomap(self):
        future = self._clear_octomap.call_async(Empty.Request())
        if not _wait_ros_client_future(future, 10.0):
            return False
        return True

    def allow_object_default_collisions(self, object_id, allow, timeout=None):
        """Set MoveIt ACM default entry for a world collision object (allow=True ignores vs all links)."""
        if timeout is None:
            timeout = self._scene_command_timeout
        cmd = {
            'action': 'allow_object_default_collisions',
            'object_id': object_id,
            'allow': bool(allow),
        }
        last = {'success': False, 'message': 'no attempt'}
        for attempt in range(self._scene_command_max_retries):
            last = self.call_json(self._scene, cmd, timeout=timeout)
            if last.get('success'):
                return last
            self._log.warn(
                f'allow_object_default_collisions({object_id}, {allow}) '
                f'attempt {attempt + 1}/{self._scene_command_max_retries}: '
                f'{last.get("message", "")}')
            if attempt + 1 < self._scene_command_max_retries:
                time.sleep(1.0)
        return last

    def list_scene_objects(self, timeout=None):
        """Return object ids tracked by scene_manager, or None if list call failed."""
        if timeout is None:
            timeout = self._scene_command_timeout
        r = self.call_json(self._scene, {'action': 'list'}, timeout=timeout)
        if not r.get('success'):
            return None
        return list(r.get('objects', []))

    def remove_collision_object(self, object_id, timeout=None):
        """Remove a world collision object from the planning scene (fast path vs ACM)."""
        if timeout is None:
            timeout = self._scene_command_timeout
        cmd = {'action': 'remove', 'id': object_id}
        last = {'success': False, 'message': 'no attempt'}
        for attempt in range(self._scene_command_max_retries):
            last = self.call_json(self._scene, cmd, timeout=timeout)
            if last.get('success'):
                return last
            self._log.warn(
                f'remove_collision_object({object_id}) '
                f'attempt {attempt + 1}/{self._scene_command_max_retries}: '
                f'{last.get("message", "")}')
            if attempt + 1 < self._scene_command_max_retries:
                time.sleep(0.5)
        return last

    def ensure_collision_object_removed(
            self, object_id, timeout=None, max_rounds=8, pause_sec=0.3):
        """Remove until scene_manager list no longer contains object_id (verified)."""
        if timeout is None:
            timeout = self._scene_command_timeout
        cmd_remove = {'action': 'remove', 'id': object_id}

        objs = self.list_scene_objects(timeout=timeout)
        if objs is not None and object_id not in objs:
            return {'success': True, 'message': f'{object_id} already absent from scene'}

        for round_idx in range(max_rounds):
            r = self.call_json(self._scene, cmd_remove, timeout=timeout)
            if not r.get('success'):
                self._log.warn(
                    f'ensure_collision_object_removed: remove {object_id} '
                    f'round {round_idx + 1}/{max_rounds}: {r.get("message", "")}')
            time.sleep(pause_sec)
            objs = self.list_scene_objects(timeout=timeout)
            if objs is None:
                self._log.warn(
                    'ensure_collision_object_removed: list failed; retrying remove')
                continue
            if object_id not in objs:
                return {'success': True, 'message': f'{object_id} removed (verified)'}

        return {
            'success': False,
            'message': (
                f'{object_id} still in scene after {max_rounds} remove/list cycles'),
        }

    def add_collision_box(
            self, object_id, position, dimensions,
            orientation=None, frame_id='link_base', timeout=None):
        """Re-add a box collision object (must match removed object geometry)."""
        if orientation is None:
            orientation = [0.0, 0.0, 0.0, 1.0]
        if timeout is None:
            timeout = self._scene_command_timeout
        cmd = {
            'action': 'add',
            'id': object_id,
            'shape': 'box',
            'frame_id': frame_id,
            'position': [float(x) for x in position],
            'dimensions': [float(x) for x in dimensions],
            'orientation': [float(x) for x in orientation],
        }
        last = {'success': False, 'message': 'no attempt'}
        for attempt in range(self._scene_command_max_retries):
            last = self.call_json(self._scene, cmd, timeout=timeout)
            if last.get('success'):
                return last
            self._log.warn(
                f'add_collision_box({object_id}) '
                f'attempt {attempt + 1}/{self._scene_command_max_retries}: '
                f'{last.get("message", "")}')
            if attempt + 1 < self._scene_command_max_retries:
                time.sleep(0.5)
        return last
