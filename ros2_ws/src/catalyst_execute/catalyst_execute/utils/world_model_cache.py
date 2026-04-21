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

    def pick_grasp_verified(
        self,
        enabled: bool,
        max_finger_opening_m: float | None,
    ) -> bool:
        """True if a pick grasp looks successful from latest /world_model gripper block.

        When ``enabled`` is False, always returns True (caller skips retry logic).

        With real ``gripper_node`` data: success if ``is_grasping`` is true, otherwise
        failure if fingers are still too open (``position_meters`` above threshold)
        or contact was not detected.

        If ``source`` is not ``gripper_node`` (sim / joint_states fallback), returns
        True so pick does not Z-retry without contact sensing.
        """
        if not enabled:
            return True
        if not self._world:
            return True
        g = self._world.get('gripper') or {}
        if g.get('source') != 'gripper_node':
            return True
        if g.get('is_grasping'):
            return True
        pm = g.get('position_meters')
        if (
            max_finger_opening_m is not None
            and pm is not None
            and float(pm) > float(max_finger_opening_m)
        ):
            return False
        # Contact not detected (e.g. miss in Z) — includes fully closed on air.
        return False
