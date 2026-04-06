"""Shared service client utilities for action servers.

Provides synchronous access to all robot services from within
MultiThreadedExecutor contexts (e.g. action server execute callbacks).
"""

import json
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool

from catalyst_interfaces.srv import JsonCommand


class RobotServiceClients:
    """Synchronous wrappers around all robot JsonCommand / SetBool / Empty services.

    Must be used from a node running inside a MultiThreadedExecutor so that
    service response callbacks can be processed while the caller blocks.
    """

    def __init__(self, node: Node, callback_group: ReentrantCallbackGroup):
        self._node = node
        self._log = node.get_logger()

        self._joint = node.create_client(
            JsonCommand, '/joint_command', callback_group=callback_group)
        self._cartesian = node.create_client(
            JsonCommand, '/cartesian_command', callback_group=callback_group)
        self._gripper = node.create_client(
            JsonCommand, '/gripper_command', callback_group=callback_group)
        self._guide_mode = node.create_client(
            JsonCommand, '/guide_mode', callback_group=callback_group)
        self._sdk = node.create_client(
            JsonCommand, '/sdk_control', callback_group=callback_group)
        self._compute_poses = node.create_client(
            JsonCommand, '/compute_poses', callback_group=callback_group)
        self._set_octomap = node.create_client(
            SetBool, '/set_octomap_enabled', callback_group=callback_group)
        self._clear_octomap = node.create_client(
            Empty, '/clear_octomap', callback_group=callback_group)

    # ── Generic helpers ──

    def wait_for_services(self, names=None, timeout=30.0):
        """Wait for required services.  *names* filters by key."""
        all_clients = {
            'joint': self._joint,
            'cartesian': self._cartesian,
            'gripper': self._gripper,
            'guide_mode': self._guide_mode,
            'sdk': self._sdk,
            'compute_poses': self._compute_poses,
            'octomap': self._set_octomap,
            'clear_octomap': self._clear_octomap,
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
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                self._log.error(f'Timeout calling {client.srv_name}')
                return {'success': False, 'message': 'Service call timeout'}
            time.sleep(0.01)

        if future.result() is None:
            return {'success': False, 'message': 'Service call returned None'}

        resp = json.loads(future.result().response)
        self._log.info(f'<- {client.srv_name}: {resp.get("message", "")}')
        return resp

    # ── Convenience methods ──

    def move_home(self, speed=0.1):
        return self.call_json(self._joint, {'pose': 'home', 'speed': speed})

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
        start = time.time()
        while not future.done():
            if time.time() - start > 10.0:
                self._log.error('Timeout setting octomap')
                return False
            time.sleep(0.01)
        return True

    def clear_octomap(self):
        future = self._clear_octomap.call_async(Empty.Request())
        start = time.time()
        while not future.done():
            if time.time() - start > 10.0:
                return False
            time.sleep(0.01)
        return True
