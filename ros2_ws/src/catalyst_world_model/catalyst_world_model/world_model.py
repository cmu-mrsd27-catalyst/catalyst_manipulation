"""
World Model Node — central state aggregator for the Catalyst Manipulator.

Subscribes to joint states, gripper state, robot status, AprilTag
detections, and TF2, then publishes a unified JSON dictionary on
/world_model at 10 Hz.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from sensor_msgs.msg import JointState

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

# Gripper fallback timeout (seconds) — if no /gripper_state received within
# this window, fall back to joint_states finger position.
GRIPPER_TOPIC_TIMEOUT = 2.0

# AprilTag detection timeout (seconds) — detections older than this are dropped.
APRILTAG_TIMEOUT = 1.0

# Tag ID to semantic frame name mapping
TAG_FRAME_NAMES = {
    14: 'tag_liquid_handler',
}


class WorldModel(Node):

    def __init__(self):
        super().__init__('world_model_node')

        # ── Internal state ──
        self._joint_positions = {}   # name -> rad
        self._joint_velocities = {}  # name -> rad/s
        self._gripper_state = None   # latest JSON dict from /gripper_state
        self._gripper_state_time = None  # Time of last /gripper_state msg
        self._finger_position = None  # from /joint_states right_finger_joint
        self._robot_status = None    # latest RobotMsg fields
        self._apriltag_detections = []  # latest AprilTagDetectionArray
        self._apriltag_time = None      # Time of last detection msg

        # ── TF2 ──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Subscriptions ──
        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10
        )
        self.create_subscription(
            String, '/gripper_state', self._gripper_state_cb, 10
        )
        self.create_subscription(
            AprilTagDetectionArray, '/detections', self._apriltag_cb, 10
        )
        if HAS_XARM_MSGS:
            self.create_subscription(
                RobotMsg, '/xarm/robot_states', self._robot_states_cb, 10
            )

        # ── Publisher ──
        self._pub = self.create_publisher(String, '/world_model', 10)

        # ── 10 Hz publish timer ──
        self.create_timer(0.1, self._publish)

        self.get_logger().info('World model node started (10 Hz)')

    # ── Callbacks ──

    def _joint_states_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self._joint_velocities[name] = msg.velocity[i]
            # Track finger position for gripper fallback
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
        }

        msg = String()
        msg.data = json.dumps(world)
        self._pub.publish(msg)

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
                'link_base', 'link_tcp', Time()
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            # Euler from quaternion (roll, pitch, yaw)
            roll, pitch, yaw = self._quat_to_euler(r.x, r.y, r.z, r.w)
            return {
                'position': {
                    'x': round(t.x, 6),
                    'y': round(t.y, 6),
                    'z': round(t.z, 6),
                },
                'orientation': {
                    'qx': round(r.x, 6),
                    'qy': round(r.y, 6),
                    'qz': round(r.z, 6),
                    'qw': round(r.w, 6),
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
        # Check if we have recent /gripper_state data
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

        # Fallback: derive from right_finger_joint in /joint_states
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
        # Return empty list if detections are stale or absent
        if (self._apriltag_time is None
                or (now - self._apriltag_time).nanoseconds / 1e9 > APRILTAG_TIMEOUT):
            return []

        tags = []
        for det in self._apriltag_detections:
            tag_id = det.id
            frame = TAG_FRAME_NAMES.get(tag_id, f'tag_{tag_id}')

            # Look up pose in link_base frame via TF2
            pose_in_base = None
            try:
                tf = self._tf_buffer.lookup_transform(
                    'link_base', frame, Time()
                )
                t = tf.transform.translation
                r = tf.transform.rotation
                roll, pitch, yaw = self._quat_to_euler(r.x, r.y, r.z, r.w)
                pose_in_base = {
                    'position': {
                        'x': round(t.x, 6),
                        'y': round(t.y, 6),
                        'z': round(t.z, 6),
                    },
                    'orientation': {
                        'qx': round(r.x, 6),
                        'qy': round(r.y, 6),
                        'qz': round(r.z, 6),
                        'qw': round(r.w, 6),
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

    @staticmethod
    def _quat_to_euler(x, y, z, w):
        """Convert quaternion to (roll, pitch, yaw) in radians."""
        # Roll (x-axis)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis)
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
