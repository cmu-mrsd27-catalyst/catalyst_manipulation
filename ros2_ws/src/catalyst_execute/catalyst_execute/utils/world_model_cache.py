"""Subscribe to /world_model JSON for gripper state (and future helpers)."""

import json

from std_msgs.msg import String


class WorldModelCache:
    """Caches latest /world_model message for conditional gripper commands."""

    def __init__(self, node, callback_group=None):
        self._node = node
        self._world = None
        kwargs = {'callback_group': callback_group} if callback_group else {}
        node.create_subscription(
            String, '/world_model', self._cb, 10, **kwargs)

    def _cb(self, msg: String):
        try:
            self._world = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def gripper_state(self) -> str:
        """open | closed | partial | unknown | unavailable"""
        if not self._world:
            return 'unknown'
        return self._world.get('gripper', {}).get('state', 'unknown')

    def gripper_open_if_needed(self, svc) -> None:
        """Open only if not already reported open."""
        s = self.gripper_state()
        if s == 'open':
            self._node.get_logger().info('[Gripper] already open — skip open')
            return
        svc.gripper('open')

    def gripper_close_if_needed(self, svc) -> None:
        """Close only if not already reported closed."""
        s = self.gripper_state()
        if s == 'closed':
            self._node.get_logger().info('[Gripper] already closed — skip close')
            return
        svc.gripper('close')
