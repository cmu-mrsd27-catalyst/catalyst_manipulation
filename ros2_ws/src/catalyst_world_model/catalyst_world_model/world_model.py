"""
World Model Node — central state aggregator for the Catalyst Manipulator.

Subscribes to joint states, gripper state, robot status, AprilTag
detections, TF2, camera, F/T sensor, octomap, and robot phase.
Publishes a unified JSON dictionary on /world_model at 10 Hz,
including phase-aware system health status.
"""

import json
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time

from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState, PointCloud2
from geometry_msgs.msg import WrenchStamped

import tf2_ros

from apriltag_msgs.msg import AprilTagDetectionArray

# Optional: only available on real hardware with xarm_msgs built
try:
    from xarm_msgs.msg import RobotMsg
    HAS_XARM_MSGS = True
except ImportError:
    HAS_XARM_MSGS = False

# Arm joint names (excludes gripper finger joints)
ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# xArm robot state mapping
ROBOT_STATE_NAMES = {
    1: 'RUNNING',
    2: 'SLEEPING',
    3: 'PAUSED',
    4: 'STOPPED',
    5: 'CONFIG_CHANGED',
}

# xArm robot mode mapping
ROBOT_MODE_NAMES = {
    0: 'POSITION',
    1: 'SERVO',
    2: 'TEACH_JOINT',
    3: 'TEACH_CART',
    4: 'VELO_JOINT',
    5: 'VELO_CART',
}

# Gripper fallback timeout (seconds)
GRIPPER_TOPIC_TIMEOUT = 2.0

# AprilTag detection timeout (seconds)
APRILTAG_TIMEOUT = 1.0

# Tag ID to semantic frame name mapping
TAG_FRAME_NAMES = {
    14: 'tag_liquid_handler',
}

# ── Health check thresholds ──
CAMERA_TIMEOUT_SEC = 2.0
FT_SENSOR_TIMEOUT_SEC = 2.0
JOINT_STATES_TIMEOUT_SEC = 1.0
OCTOMAP_TIMEOUT_SEC = 5.0
ROBOT_PHASE_TIMEOUT_SEC = 5.0
COLLISION_FORCE_THRESHOLD_N = 15.0
CONSECUTIVE_FAIL_THRESHOLD = 5  # check must fail this many times in a row to be unhealthy
# Phases where specific checks are suppressed (expected transient states)
SUPPRESS_ROBOT_STATE = {
    'GUIDE_MODE', 'CLOSING_GRIPPER', 'DISABLING_GUIDE_MODE',
    'SDK_CONNECTING', 'CORNER_REGISTRATION', 'POSITION_CORRECTION',
    'FORCE_DESCENT', 'ADMITTANCE', 'SDK_RETRACT', 'RELEASING', 'CLEANUP',
    'WAITING_FOR_SERVICES',
}
SUPPRESS_CONTROLLERS = SUPPRESS_ROBOT_STATE  # same set
SUPPRESS_FT_SENSOR = SUPPRESS_ROBOT_STATE    # F/T broadcaster stops during controller transitions
SUPPRESS_OCTOMAP = {
    'PICKING', 'GUIDE_MODE', 'CLOSING_GRIPPER', 'DISABLING_GUIDE_MODE',
    'PRE_PLACING',
    'SDK_CONNECTING', 'CORNER_REGISTRATION', 'POSITION_CORRECTION',
    'FORCE_DESCENT', 'ADMITTANCE', 'SDK_RETRACT', 'RELEASING', 'CLEANUP',
    'WAITING_FOR_SERVICES',
}
SUPPRESS_COLLISION_FORCE = {
    'PICKING', 'GUIDE_MODE', 'CLOSING_GRIPPER', 'DISABLING_GUIDE_MODE',
    'SDK_CONNECTING', 'CORNER_REGISTRATION', 'POSITION_CORRECTION',
    'FORCE_DESCENT', 'ADMITTANCE', 'SDK_RETRACT', 'RELEASING', 'CLEANUP',
    'WAITING_FOR_SERVICES',
    'OPENING_GRIPPER',
}

# Defaults; overridden by launch (demo.launch.py loads config/world_model.yaml) or ros2 param CLI.
_WORLD_MODEL_PARAM_DEFAULTS = {
    'robot_ip': '192.168.1.212',
    'ft_sensor_ip': '192.168.2.1',
    'camera_topic': '/camera/camera/color/image_raw',
    'ft_topic': '/force_torque_sensor_broadcaster/wrench',
    'octomap_topic': '/filtered_pointcloud',
    'health.check_camera': True,
    'health.check_gripper': False,
    'health.check_ft_sensor': False,
    'health.check_robot_state': False,
    'health.check_controllers': False,
    'health.check_octomap': True,
    'health.check_collision_force': True,
    'health.check_joint_temperature': False,
}

def _load_share_yaml_ros_params():
    """Return (ros__parameters dict, path used).

    Order: optional CATALYST_WORLD_MODEL_YAML (absolute path in container, e.g.
    /root/ros2_ws/src/catalyst_world_model/config/world_model.yaml), else
    install share from ament index. No host-specific paths are hardcoded.
    """
    candidates = []
    env_yaml = os.environ.get('CATALYST_WORLD_MODEL_YAML', '').strip()
    if env_yaml:
        candidates.append(env_yaml)
    try:
        share = get_package_share_directory('catalyst_world_model')
        candidates.append(os.path.join(share, 'config', 'world_model.yaml'))
    except Exception:
        pass
    fallback = (
        candidates[-1]
        if candidates
        else 'share/catalyst_world_model/config/world_model.yaml'
    )
    for path in candidates:
        if not path or not os.path.isfile(path):
            continue
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            params = (data.get('world_model_node') or {}).get('ros__parameters') or {}
            return params, path
        except Exception:
            continue
    return {}, fallback


class WorldModel(Node):

    def __init__(self):
        super().__init__('world_model_node')

        # Declare defaults; launch may inject YAML. Re-apply share/world_model.yaml with
        # set_parameters so the on-disk file always wins (fixes launch/declare ordering and
        # stale defaults when the install share is the only copy updated by rebuild).
        for key, default in _WORLD_MODEL_PARAM_DEFAULTS.items():
            self.declare_parameter(key, default)

        share_params, share_yaml_path = _load_share_yaml_ros_params()
        overlay_keys = [
            k for k in share_params if k in _WORLD_MODEL_PARAM_DEFAULTS
        ]
        if overlay_keys:
            to_set = [Parameter(k, value=share_params[k]) for k in overlay_keys]
            results = self.set_parameters(to_set)
            for r in results:
                if not r.successful:
                    self.get_logger().warning(
                        f'world_model.yaml ({share_yaml_path}): could not set '
                        f'{r.name}: {r.reason}'
                    )
            self.get_logger().info(
                f'Overlaid {len(overlay_keys)} parameters from {share_yaml_path}'
            )
        else:
            self.get_logger().warning(
                f'No world_model.yaml parameters overlaid (path={share_yaml_path!r})'
            )

        health_keys = [k for k in _WORLD_MODEL_PARAM_DEFAULTS if k.startswith('health.')]
        self.get_logger().info(
            'Health toggles (effective): '
            + ', '.join(f'{k}={self.get_parameter(k).value}' for k in sorted(health_keys))
        )


        robot_ip = self.get_parameter('robot_ip').value
        ft_ip = self.get_parameter('ft_sensor_ip').value
        camera_topic = self.get_parameter('camera_topic').value
        ft_topic = self.get_parameter('ft_topic').value
        octomap_topic = self.get_parameter('octomap_topic').value

        # ── Internal state ──
        self._joint_positions = {}
        self._joint_velocities = {}
        self._joint_states_time = None
        self._gripper_state = None
        self._gripper_state_time = None
        self._finger_position = None
        self._robot_status = None
        self._apriltag_detections = []
        self._apriltag_time = None

        # Health-related state
        self._camera_time = None
        self._ft_time = None
        self._ft_wrench = None  # latest [fx, fy, fz]
        self._octomap_time = None
        self._robot_phase = 'IDLE'
        self._robot_phase_time = None

        # Consecutive failure counters (check must fail N times in a row)
        self._fail_counts = {
            'camera': 0,
            'gripper': 0,
            'ft_sensor': 0,
            'robot_state': 0,
            'controllers': 0,
            'octomap': 0,
            'collision_force': 0,
        }

        # ── TF2 ──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Subscriptions ──
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)
        self.create_subscription(
            String, '/gripper_state', self._gripper_state_cb, 10)
        self.create_subscription(
            AprilTagDetectionArray, '/detections', self._apriltag_cb, 10)
        if HAS_XARM_MSGS:
            self.create_subscription(
                RobotMsg, '/xarm/robot_states', self._robot_states_cb, 10)

        # Health-related subscriptions
        self.create_subscription(
            Image, camera_topic, self._camera_cb, 10)
        self.create_subscription(
            WrenchStamped, ft_topic, self._ft_cb, 10)
        self.create_subscription(
            PointCloud2, octomap_topic, self._octomap_cb, 10)
        self.create_subscription(
            String, '/robot_phase', self._robot_phase_cb, 10)

        # Network checker removed — F/T and joint_states freshness checks
        # already cover network reachability.

        # ── Publisher ──
        self._pub = self.create_publisher(String, '/world_model', 10)

        # ── 10 Hz publish timer ──
        self.create_timer(0.1, self._publish)

        self.get_logger().info(
            f'World model node started (10 Hz) — '
            f'camera: {camera_topic}, ft: {ft_topic}, '
            f'octomap: {octomap_topic}')

    # ── Existing callbacks ──

    def _joint_states_cb(self, msg: JointState):
        self._joint_states_time = self.get_clock().now()
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self._joint_velocities[name] = msg.velocity[i]
            if name == 'right_finger_joint' and i < len(msg.position):
                self._finger_position = msg.position[i]

    def _gripper_state_cb(self, msg: String):
        try:
            self._gripper_state = json.loads(msg.data)
            self._gripper_state_time = self.get_clock().now()
        except json.JSONDecodeError:
            pass

    def _apriltag_cb(self, msg: AprilTagDetectionArray):
        self._apriltag_detections = msg.detections
        self._apriltag_time = self.get_clock().now()

    def _robot_states_cb(self, msg):
        self._robot_status = {
            'state': msg.state,
            'state_name': ROBOT_STATE_NAMES.get(msg.state, 'UNKNOWN'),
            'mode': msg.mode,
            'mode_name': ROBOT_MODE_NAMES.get(msg.mode, 'UNKNOWN'),
            'is_teach_mode': msg.mode in (2, 3),
            'error': msg.err,
            'warning': msg.warn,
            'source': 'xarm_driver',
        }

    # ── Health-related callbacks ──

    def _camera_cb(self, msg):
        self._camera_time = self.get_clock().now()

    def _ft_cb(self, msg: WrenchStamped):
        self._ft_time = self.get_clock().now()
        w = msg.wrench
        self._ft_wrench = [w.force.x, w.force.y, w.force.z]

    def _octomap_cb(self, msg):
        self._octomap_time = self.get_clock().now()

    def _robot_phase_cb(self, msg: String):
        self._robot_phase = msg.data
        self._robot_phase_time = self.get_clock().now()

    # ── Publish ──

    def _publish(self):
        now = self.get_clock().now()
        timestamp = now.nanoseconds / 1e9

        world = {
            'timestamp': timestamp,
            'joint_states': self._build_joint_states(),
            'tcp_pose': self._build_tcp_pose(),
            'gripper': self._build_gripper(now),
            'apriltags': self._build_apriltags(now),
            'robot_status': self._build_robot_status(),
            'robot_phase': self._robot_phase,
            'system_health': self._build_system_health(now),
        }

        msg = String()
        msg.data = json.dumps(world)
        self._pub.publish(msg)

    # ── Existing builders ──

    def _build_joint_states(self):
        positions_rad = [self._joint_positions.get(n, 0.0) for n in ARM_JOINT_NAMES]
        positions_deg = [math.degrees(r) for r in positions_rad]
        velocities = [self._joint_velocities.get(n, 0.0) for n in ARM_JOINT_NAMES]
        return {
            'names': ARM_JOINT_NAMES,
            'positions_deg': [round(v, 4) for v in positions_deg],
            'positions_rad': [round(v, 6) for v in positions_rad],
            'velocities': [round(v, 6) for v in velocities],
        }

    def _build_tcp_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'link_base', 'link_tcp', Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            roll, pitch, yaw = self._quat_to_euler(r.x, r.y, r.z, r.w)
            return {
                'position': {
                    'x': round(t.x, 6), 'y': round(t.y, 6), 'z': round(t.z, 6),
                },
                'orientation': {
                    'qx': round(r.x, 6), 'qy': round(r.y, 6),
                    'qz': round(r.z, 6), 'qw': round(r.w, 6),
                },
                'orientation_euler_deg': {
                    'roll': round(math.degrees(roll), 4),
                    'pitch': round(math.degrees(pitch), 4),
                    'yaw': round(math.degrees(yaw), 4),
                },
            }
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return {
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0},
                'orientation_euler_deg': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            }

    def _build_gripper(self, now):
        if (self._gripper_state is not None
                and self._gripper_state_time is not None):
            age = (now - self._gripper_state_time).nanoseconds / 1e9
            if age < GRIPPER_TOPIC_TIMEOUT:
                gs = self._gripper_state
                return {
                    'position_meters': gs.get('position_meters'),
                    'state': gs.get('state', 'unknown'),
                    'is_grasping': gs.get('is_grasping', False),
                    'torque': gs.get('torque'),
                    'source': 'gripper_node',
                }

        pos = self._finger_position
        if pos is not None:
            if pos > 0.03:
                state = 'open'
            elif pos < 0.005:
                state = 'closed'
            else:
                state = 'partial'
            return {
                'position_meters': round(pos, 6),
                'state': state,
                'is_grasping': False,
                'torque': None,
                'source': 'joint_states_fallback',
            }

        return {
            'position_meters': None,
            'state': 'unknown',
            'is_grasping': False,
            'torque': None,
            'source': 'unavailable',
        }

    def _build_apriltags(self, now):
        if (self._apriltag_time is None
                or (now - self._apriltag_time).nanoseconds / 1e9 > APRILTAG_TIMEOUT):
            return []

        tags = []
        for det in self._apriltag_detections:
            tag_id = det.id
            frame = TAG_FRAME_NAMES.get(tag_id, f'tag_{tag_id}')

            pose_in_base = None
            try:
                tf = self._tf_buffer.lookup_transform(
                    'link_base', frame, Time())
                t = tf.transform.translation
                r = tf.transform.rotation
                roll, pitch, yaw = self._quat_to_euler(r.x, r.y, r.z, r.w)
                pose_in_base = {
                    'position': {
                        'x': round(t.x, 6), 'y': round(t.y, 6), 'z': round(t.z, 6),
                    },
                    'orientation': {
                        'qx': round(r.x, 6), 'qy': round(r.y, 6),
                        'qz': round(r.z, 6), 'qw': round(r.w, 6),
                    },
                    'orientation_euler_deg': {
                        'roll': round(math.degrees(roll), 4),
                        'pitch': round(math.degrees(pitch), 4),
                        'yaw': round(math.degrees(yaw), 4),
                    },
                }
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass

            tags.append({
                'id': tag_id,
                'frame': frame,
                'pose_in_base': pose_in_base,
            })

        return tags

    def _build_robot_status(self):
        if self._robot_status is not None:
            return self._robot_status
        return {
            'state_name': 'unknown',
            'source': 'unavailable',
        }

    # ── System health ──

    def _health_enabled(self, param_name: str) -> bool:
        """YAML/launch sometimes provide string flags; treat 'false' as disabled."""
        v = self.get_parameter(param_name).value
        if isinstance(v, bool):
            return v
        if isinstance(v, str):
            return v.strip().lower() not in ('false', '0', 'no', 'off', '')
        return bool(v)

    def _health_disabled_result(self, counter_key: str, msg: str = ''):
        """Skip a check: ok for aggregation, disabled flag for introspection."""
        self._fail_counts[counter_key] = 0
        return {
            'ok': True,
            'disabled': True,
            'suppressed': False,
            'msg': msg or 'Check disabled via parameter',
        }

    def _build_system_health(self, now):
        phase = self._robot_phase
        checks = {}

        # Camera — is depth/color image arriving?
        if not self._health_enabled('health.check_camera'):
            checks['camera'] = self._health_disabled_result('camera')
        else:
            checks['camera'] = self._debounced_check(
                'camera',
                self._check_freshness(
                    self._camera_time, now, CAMERA_TIMEOUT_SEC,
                    'Camera not publishing'))

        # Gripper — is gripper node publishing /gripper_state?
        if not self._health_enabled('health.check_gripper'):
            checks['gripper'] = self._health_disabled_result('gripper')
        else:
            checks['gripper'] = self._debounced_check(
                'gripper',
                self._check_freshness(
                    self._gripper_state_time, now, GRIPPER_TOPIC_TIMEOUT,
                    'Gripper node not publishing'))

        # F/T sensor — is it publishing?
        # Suppressed during controller transitions (broadcaster stops temporarily)
        if not self._health_enabled('health.check_ft_sensor'):
            checks['ft_sensor'] = self._health_disabled_result('ft_sensor')
        else:
            ft_suppressed = phase in SUPPRESS_FT_SENSOR
            ft_raw = self._check_freshness(
                self._ft_time, now, FT_SENSOR_TIMEOUT_SEC, 'F/T sensor not publishing')
            if ft_suppressed:
                self._fail_counts['ft_sensor'] = 0
                checks['ft_sensor'] = {'ok': True, 'suppressed': True, 'msg': ft_raw['msg']}
            else:
                checks['ft_sensor'] = self._debounced_check('ft_sensor', ft_raw)

        # Robot state — xArm error code
        if not self._health_enabled('health.check_robot_state'):
            checks['robot_state'] = self._health_disabled_result('robot_state')
        else:
            robot_ok = True
            robot_msg = ''
            suppressed = phase in SUPPRESS_ROBOT_STATE
            if not suppressed and self._robot_status is not None:
                if self._robot_status.get('error', 0) != 0:
                    robot_ok = False
                    robot_msg = f'xArm error code: {self._robot_status["error"]}'
            if suppressed:
                self._fail_counts['robot_state'] = 0
                checks['robot_state'] = {'ok': True, 'suppressed': True, 'msg': robot_msg}
            else:
                checks['robot_state'] = self._debounced_check(
                    'robot_state',
                    {'ok': robot_ok, 'suppressed': False, 'msg': robot_msg})

        # Controllers — using joint_states freshness as proxy
        if not self._health_enabled('health.check_controllers'):
            checks['controllers'] = self._health_disabled_result('controllers')
        else:
            ctrl_suppressed = phase in SUPPRESS_CONTROLLERS
            ctrl_ok = True
            ctrl_msg = ''
            if not ctrl_suppressed:
                if (self._joint_states_time is None or
                        (now - self._joint_states_time).nanoseconds / 1e9 > JOINT_STATES_TIMEOUT_SEC):
                    ctrl_ok = False
                    ctrl_msg = 'Joint states not publishing (controllers may be down)'
            if ctrl_suppressed:
                self._fail_counts['controllers'] = 0
                checks['controllers'] = {'ok': True, 'suppressed': True, 'msg': ctrl_msg}
            else:
                checks['controllers'] = self._debounced_check(
                    'controllers',
                    {'ok': ctrl_ok, 'suppressed': False, 'msg': ctrl_msg})

        # Octomap — is filtered pointcloud arriving?
        if not self._health_enabled('health.check_octomap'):
            checks['octomap'] = self._health_disabled_result('octomap')
        else:
            omap_suppressed = phase in SUPPRESS_OCTOMAP
            omap_raw = self._check_freshness(
                self._octomap_time, now, OCTOMAP_TIMEOUT_SEC,
                'Octomap input not publishing')
            if omap_suppressed:
                self._fail_counts['octomap'] = 0
                checks['octomap'] = {'ok': True, 'suppressed': True, 'msg': omap_raw['msg']}
            else:
                checks['octomap'] = self._debounced_check('octomap', omap_raw)

        # Collision force — unexpected force during transit
        if not self._health_enabled('health.check_collision_force'):
            checks['collision_force'] = self._health_disabled_result('collision_force')
        else:
            force_suppressed = phase in SUPPRESS_COLLISION_FORCE
            force_ok = True
            force_msg = ''
            if not force_suppressed and self._ft_wrench is not None:
                max_force = max(abs(f) for f in self._ft_wrench)
                if max_force > COLLISION_FORCE_THRESHOLD_N:
                    force_ok = False
                    force_msg = (f'Unexpected force: {max_force:.1f} N '
                                 f'(threshold: {COLLISION_FORCE_THRESHOLD_N} N)')
            if force_suppressed:
                self._fail_counts['collision_force'] = 0
                checks['collision_force'] = {'ok': True, 'suppressed': True, 'msg': force_msg}
            else:
                checks['collision_force'] = self._debounced_check(
                    'collision_force',
                    {'ok': force_ok, 'suppressed': False, 'msg': force_msg})

        # TODO: Joint temperature — needs xArm SDK topic or service
        if not self._health_enabled('health.check_joint_temperature'):
            checks['joint_temperature'] = self._health_disabled_result(
                'joint_temperature', 'Check disabled via parameter')
        else:
            checks['joint_temperature'] = {
                'ok': True,
                'suppressed': False,
                'msg': '',
            }

        # Overall healthy = all checks pass
        healthy = all(c['ok'] for c in checks.values())


        return {
            'healthy': healthy,
            'phase': phase,
            'checks': checks,
        }

    def _debounced_check(self, name, raw_check):
        """Only report failure after CONSECUTIVE_FAIL_THRESHOLD consecutive failures."""
        if raw_check['ok']:
            self._fail_counts[name] = 0
            return raw_check
        self._fail_counts[name] += 1
        if self._fail_counts[name] >= CONSECUTIVE_FAIL_THRESHOLD:
            return raw_check
        # Not enough consecutive failures yet — report OK
        return {'ok': True, 'suppressed': False, 'msg': ''}

    def _check_freshness(self, last_time, now, timeout_sec, fail_msg):
        """Helper: check if a topic has published recently."""
        if last_time is None:
            return {'ok': False, 'suppressed': False, 'msg': fail_msg}
        age = (now - last_time).nanoseconds / 1e9
        if age > timeout_sec:
            return {'ok': False, 'suppressed': False,
                    'msg': f'{fail_msg} (age: {age:.1f}s)'}
        return {'ok': True, 'suppressed': False, 'msg': ''}

    @staticmethod
    def _quat_to_euler(x, y, z, w):
        """Convert quaternion to (roll, pitch, yaw) in radians."""
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = WorldModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
