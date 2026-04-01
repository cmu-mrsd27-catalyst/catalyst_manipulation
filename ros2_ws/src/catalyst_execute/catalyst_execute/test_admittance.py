"""
Admittance control test node for Catalyst Manipulator.

Subscribes to an F/T sensor (WrenchStamped), computes compliance velocities
using a simple admittance law (F = D * v), and streams them to MoveIt Servo
as TwistStamped commands.

A constant slow downward velocity is added as baseline so the arm drifts
down.  Push the end-effector with your hand and the arm should yield.

Usage (after demo.launch.py is running):
  ros2 launch catalyst_bringup admittance_test.launch.py ft_topic:=/your/wrench/topic
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped, WrenchStamped
from std_srvs.srv import Trigger
from moveit_msgs.srv import ServoCommandType
import tf2_ros


class AdmittanceControlNode(Node):

    def __init__(self):
        super().__init__('admittance_control_node')
        cb_group = ReentrantCallbackGroup()

        # ── Parameters ──
        self.declare_parameter('ft_topic', '/ft_data')
        self.declare_parameter('sensor_frame', '')  # empty = skip TF transform
        self.declare_parameter('command_frame', 'link_base')
        self.declare_parameter('damping_linear', [80.0, 80.0, 80.0])   # N/(m/s)
        self.declare_parameter('damping_angular', [10.0, 10.0, 10.0])  # Nm/(rad/s)
        self.declare_parameter('force_deadband', 3.0)   # N
        self.declare_parameter('torque_deadband', 0.5)   # Nm
        self.declare_parameter('max_linear_vel', 0.15)   # m/s
        self.declare_parameter('max_angular_vel', 0.5)   # rad/s
        self.declare_parameter('insert_velocity_z', 0.0) # m/s  (negative = down)
        self.declare_parameter('force_safety_limit', 80.0)  # N — stop if exceeded
        self.declare_parameter('control_rate', 100.0)     # Hz

        self._ft_topic = self.get_parameter('ft_topic').value
        self._sensor_frame = self.get_parameter('sensor_frame').value
        self._cmd_frame = self.get_parameter('command_frame').value
        self._D_lin = np.array(self.get_parameter('damping_linear').value)
        self._D_ang = np.array(self.get_parameter('damping_angular').value)
        self._force_db = self.get_parameter('force_deadband').value
        self._torque_db = self.get_parameter('torque_deadband').value
        self._max_lin = self.get_parameter('max_linear_vel').value
        self._max_ang = self.get_parameter('max_angular_vel').value
        self._v_insert = self.get_parameter('insert_velocity_z').value
        self._f_safety = self.get_parameter('force_safety_limit').value
        self._rate = self.get_parameter('control_rate').value

        # ── TF (optional wrench rotation) ──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Pub / Sub ──
        self._twist_pub = self.create_publisher(
            TwistStamped, '/servo_server/delta_twist_cmds', 10
        )
        self._wrench_sub = self.create_subscription(
            WrenchStamped, self._ft_topic, self._wrench_cb, 10,
            callback_group=cb_group,
        )

        self._latest_wrench = None
        self._servo_ready = False

        # ── Kick off servo init after delay (Servo needs time to receive robot state) ──
        self.create_timer(10.0, self._init_servo, callback_group=cb_group)

        self.get_logger().info(
            f'Admittance node ready — waiting for Servo. '
            f'FT topic: {self._ft_topic}, insert_vz: {self._v_insert} m/s'
        )

    # ── Servo initialisation ──────────────────────────────────────────
    def _init_servo(self):
        """Call start_servo + switch to TWIST mode, then start control loop."""
        if self._servo_ready:
            return

        # 1. Start servo
        start_cli = self.create_client(Trigger, '/servo_server/start_servo')
        if not start_cli.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('start_servo service not available — is Servo node running?')
            return
        future = start_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.get_logger().info(f'start_servo: {future.result().message}')
        else:
            self.get_logger().error('start_servo call failed')
            return

        # 2. Switch to TWIST command type
        switch_cli = self.create_client(
            ServoCommandType, '/servo_server/switch_command_type'
        )
        if not switch_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('switch_command_type service not available')
            return
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        future = switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None and future.result().success:
            self.get_logger().info('Switched Servo to TWIST mode')
        else:
            self.get_logger().error('Failed to switch to TWIST mode')
            return

        self._servo_ready = True
        self.get_logger().info('Servo ready — starting admittance control loop')

        # Start the control timer
        period = 1.0 / self._rate
        self.create_timer(period, self._control_loop)

    # ── Callbacks ─────────────────────────────────────────────────────
    def _wrench_cb(self, msg: WrenchStamped):
        self._latest_wrench = msg

    # ── Control loop ──────────────────────────────────────────────────
    def _control_loop(self):
        if not self._servo_ready:
            return

        # Default: zero wrench
        force = np.zeros(3)
        torque = np.zeros(3)

        if self._latest_wrench is not None:
            w = self._latest_wrench.wrench
            force = np.array([w.force.x, w.force.y, w.force.z])
            torque = np.array([w.torque.x, w.torque.y, w.torque.z])

            # Optionally rotate wrench from sensor frame into command frame
            if self._sensor_frame:
                force, torque = self._transform_wrench(force, torque)

        # Safety check
        f_mag = np.linalg.norm(force)
        if f_mag > self._f_safety:
            self.get_logger().warn(
                f'Force magnitude {f_mag:.1f} N exceeds safety limit '
                f'{self._f_safety:.1f} N — sending zero twist'
            )
            self._publish_zero()
            return

        # Apply deadband
        force = np.where(np.abs(force) > self._force_db, force, 0.0)
        torque = np.where(np.abs(torque) > self._torque_db, torque, 0.0)

        # Admittance law:  v = F / D
        v_lin = force / self._D_lin
        v_ang = torque / self._D_ang

        # Add baseline insertion velocity
        v_lin[2] += self._v_insert

        # Clamp
        v_lin = np.clip(v_lin, -self._max_lin, self._max_lin)
        v_ang = np.clip(v_ang, -self._max_ang, self._max_ang)

        # Publish
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._cmd_frame
        msg.twist.linear.x = float(v_lin[0])
        msg.twist.linear.y = float(v_lin[1])
        msg.twist.linear.z = float(v_lin[2])
        msg.twist.angular.x = float(v_ang[0])
        msg.twist.angular.y = float(v_ang[1])
        msg.twist.angular.z = float(v_ang[2])
        self._twist_pub.publish(msg)

    # ── Wrench frame rotation ─────────────────────────────────────────
    def _transform_wrench(self, force, torque):
        """Rotate force/torque vectors from sensor_frame to command_frame."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._cmd_frame, self._sensor_frame, rclpy.time.Time()
            )
            q = tf.transform.rotation
            R = self._quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
            return R @ force, R @ torque
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn_throttle(
                self.get_clock(), 5000,
                f'Cannot transform wrench ({e}), using raw values',
            )
            return force, torque

    @staticmethod
    def _quat_to_rotation_matrix(x, y, z, w):
        """Quaternion (x,y,z,w) → 3×3 rotation matrix."""
        return np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
        ])

    # ── Helpers ───────────────────────────────────────────────────────
    def _publish_zero(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._cmd_frame
        self._twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down — sending zero twist')
    finally:
        node._publish_zero()
        node.destroy_node()
        rclpy.try_shutdown()
