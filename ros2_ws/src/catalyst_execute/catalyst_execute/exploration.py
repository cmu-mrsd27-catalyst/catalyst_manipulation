"""
AprilTag exploration for the Catalyst Manipulator.

Sweeps the base joint (joint1) through a range while holding the other
joints at a fixed "look" pose, checking /world_model at each step for
a target AprilTag detection.

Usage (from another script that already has a running ROS node):

    from catalyst_execute.exploration import TagExplorer

    explorer = TagExplorer(node, joint_client, world_model_getter)
    tag_pose = explorer.search()
    if tag_pose is not None:
        # tag_pose is a dict with 'position' and 'orientation' in link_base
        ...
"""

import json
import time

import rclpy
from catalyst_interfaces.srv import JsonCommand

from catalyst_execute.utils.execute_config import section


def _exploration_defaults():
    ex = section('exploration')
    joints = ex.get(
        'explore_joints_deg',
        [101.8256, -63.9126, -19.2997, 176.077, -43.6008, 188.82])
    return {
        'explore_joints': list(joints),
        'sweep_start': float(ex.get('sweep_start', 91.9783)),
        'sweep_end': float(ex.get('sweep_end', 135.9783)),
        'sweep_step': float(ex.get('sweep_step', 10.0)),
        'tag_frame': ex.get('default_tag_frame', 'tag_liquid_handler'),
        'settle_time': float(ex.get('settle_time', 0.5)),
        'fresh_read_timeout': float(ex.get('fresh_read_timeout', 2.0)),
        'joint_move_timeout_sec': float(ex.get('joint_move_timeout_sec', 30.0)),
        'move_speed': float(ex.get('default_move_speed', 0.1)),
    }


class TagExplorer:
    """Sweeps joint1 to find an AprilTag, using an existing ROS node."""

    def __init__(self, node, joint_client, world_model_getter,
                 tag_frame=None,
                 explore_joints=None,
                 sweep_start=None,
                 sweep_end=None,
                 sweep_step=None,
                 move_speed=None,
                 settle_time=None,
                 fresh_read_timeout=None,
                 joint_move_timeout_sec=None):
        """
        Args:
            node: An rclpy Node (used for logging and spinning).
            joint_client: A ready rclpy service client for /joint_command.
            world_model_getter: A callable that returns the latest
                                (timestamp, world_model_dict) tuple.
                                timestamp is the 'timestamp' field from the
                                world_model JSON (epoch seconds).
                                The caller is responsible for subscribing to
                                /world_model and keeping it updated.
            tag_frame: The tag frame name to search for.
            explore_joints: Base 6-joint pose in degrees (joint1 will be
                            overridden during sweep).
            sweep_start: Start angle for joint1 sweep (degrees).
            sweep_end: End angle for joint1 sweep (degrees).
            sweep_step: Step size for joint1 sweep (degrees).
            move_speed: Velocity scaling factor for joint moves (0.01-1.0).
            settle_time: Seconds to wait after each move before checking.
            fresh_read_timeout: Max seconds to wait for a fresh world_model.
            joint_move_timeout_sec: Timeout for each joint_command call.
        """
        d = _exploration_defaults()
        self._node = node
        self._joint_client = joint_client
        self._get_world_model = world_model_getter
        self._tag_frame = tag_frame if tag_frame is not None else d['tag_frame']
        self._explore_joints = list(
            explore_joints if explore_joints is not None else d['explore_joints'])
        self._sweep_start = float(sweep_start if sweep_start is not None else d['sweep_start'])
        self._sweep_end = float(sweep_end if sweep_end is not None else d['sweep_end'])
        self._sweep_step = float(sweep_step if sweep_step is not None else d['sweep_step'])
        self._move_speed = float(move_speed if move_speed is not None else d['move_speed'])
        self._settle_time = float(settle_time if settle_time is not None else d['settle_time'])
        self._fresh_read_timeout = float(
            fresh_read_timeout if fresh_read_timeout is not None else d['fresh_read_timeout'])
        self._joint_move_timeout_sec = float(
            joint_move_timeout_sec
            if joint_move_timeout_sec is not None else d['joint_move_timeout_sec'])

    def search(self):
        """Sweep joint1 and return the first detected tag pose, or None.

        Returns:
            dict with 'position' and 'orientation' keys (in link_base frame),
            or None if the tag was not found anywhere in the sweep.
        """
        logger = self._node.get_logger()
        logger.info(
            f'Starting tag exploration: joint1 {self._sweep_start} to '
            f'{self._sweep_end} deg in {self._sweep_step} deg steps, '
            f'looking for "{self._tag_frame}"'
        )

        j1 = self._sweep_start
        count = 0
        while j1 <= self._sweep_end + 1e-6:
            joints = self._explore_joints.copy()
            joints[0] = j1

            logger.info(f'Exploring joint1 = {j1:.1f} deg')
            if not self._move_joints(joints):
                logger.warn(f'Joint move failed at joint1 = {j1:.1f} deg, skipping')
                j1 += self._sweep_step
                continue

            # Wait for the robot to fully settle and vision to refresh
            time.sleep(self._settle_time)

            # Record cutoff time *after* settling
            t_cutoff = self._node.get_clock().now().nanoseconds / 1e9

            tag_pose = self._read_fresh_tag(t_cutoff)
            if tag_pose is not None and count > 0:
                logger.info(
                    f'Tag "{self._tag_frame}" found at joint1 = {j1:.1f} deg'
                )
                return tag_pose, j1

            j1 += self._sweep_step
            count += 1

        logger.warn(f'Tag "{self._tag_frame}" not found in sweep range')
        return None, None

    def _move_joints(self, joints_deg):
        """Send a joint command and return True on success."""
        req = JsonCommand.Request()
        req.command = json.dumps({
            'joints': joints_deg,
            'speed': self._move_speed,
        })
        future = self._joint_client.call_async(req)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=self._joint_move_timeout_sec)
        if future.result() is None:
            return False
        resp = json.loads(future.result().response)
        return resp.get('success', False)

    def _read_fresh_tag(self, t_cutoff):
        """Spin until a world_model message newer than t_cutoff arrives,
        then check it for the target tag.

        Returns:
            pose dict with 'position' and 'orientation', or None.
        """
        deadline = time.monotonic() + self._fresh_read_timeout

        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            ts, wm = self._get_world_model()
            if wm is None or ts is None:
                continue
            if ts < t_cutoff:
                continue
            # This message arrived after we settled — trust it
            for tag in wm.get('apriltags', []):
                if (tag['frame'] == self._tag_frame
                        and tag.get('pose_in_base') is not None):
                    return tag['pose_in_base']
            # Fresh message arrived but tag not in it — not visible here
            return None

        self._node.get_logger().warn(
            'Timed out waiting for fresh world_model message'
        )
        return None
