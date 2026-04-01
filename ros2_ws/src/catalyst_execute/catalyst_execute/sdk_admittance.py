#!/usr/bin/env python3
"""SDK-based admittance control for well-plate insertion.

Bypasses ros2_control entirely for the compliance loop:
  OnRobot F/T sensor (UDP) → admittance math → xArm SDK servo commands

This eliminates the ~1-2s latency caused by the ros2_control → xArm hardware
interface pipeline (firmware motion smoothing on set_servo_angle_j).

Usage (after demo.launch.py is running):
  ros2 run catalyst_execute sdk_admittance --ros-args -p robot_ip:=192.168.1.212

The node:
  1. Deactivates xarm6_traj_controller (keeps broadcasters alive)
  2. Connects to xArm via Python SDK in servo mode
  3. Reads F/T data directly from OnRobot sensor via UDP
  4. Runs a tight admittance loop: F → velocity → incremental servo commands
  5. On exit, restores trajectory controller

Press Ctrl+C to stop and restore normal operation.
"""

import math
import socket
import struct
import threading
import time
from xml.etree import ElementTree

import numpy as np
import requests
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
from xarm.wrapper import XArmAPI

# xArm mode/state constants
XARM_MODE_SERVO = 1
XARM_MODE_SERVO_CART_AA = 6
XARM_STATE_START = 0
XARM_STATE_STOP = 4

# OnRobot RDT protocol constants
ONROBOT_PORT = 49152
RDT_HEADER = 0x1234
RDT_CMD_START = 0x0002
RDT_CMD_STOP = 0x0000
RDT_CMD_BIAS = 0x0042
RDT_RECORD_SIZE = 36


class OnRobotFTReader:
    """Reads force/torque data directly from OnRobot sensor via UDP."""

    def __init__(self, ip_address: str):
        self.ip = ip_address
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)
        self.sock.connect((ip_address, ONROBOT_PORT))

        # Fetch calibration scales from sensor
        self.force_scale, self.torque_scale = self._get_calibration()

        # Latest reading (thread-safe)
        self._lock = threading.Lock()
        self._ft = np.zeros(6)  # [Fx, Fy, Fz, Tx, Ty, Tz]
        self._running = False
        self._thread = None

    def _get_calibration(self) -> tuple:
        """Fetch calibration factors from sensor HTTP API."""
        url = f"http://{self.ip}/netftcalapi.xml"
        resp = requests.get(url, timeout=5.0)
        root = ElementTree.fromstring(resp.content)

        calcpf = float(root.find('.//calcpf').text)
        calcpt = float(root.find('.//calcpt').text)
        return 1.0 / calcpf, 1.0 / calcpt

    def _pack_command(self, command: int, sample_count: int = 0) -> bytes:
        return struct.pack('>HHI', RDT_HEADER, command, sample_count)

    def _unpack_record(self, data: bytes) -> np.ndarray:
        """Parse 36-byte RDT record into [Fx,Fy,Fz,Tx,Ty,Tz] in N and Nm."""
        raw = struct.unpack('>3I6i', data)
        # raw[0:3] = rdt_seq, ft_seq, status
        # raw[3:9] = Fx, Fy, Fz, Tx, Ty, Tz (raw counts)
        ft = np.zeros(6)
        for i in range(3):
            ft[i] = raw[3 + i] * self.force_scale
        for i in range(3):
            ft[3 + i] = raw[6 + i] * self.torque_scale
        return ft

    def start(self):
        """Start continuous UDP streaming in a background thread."""
        self.sock.send(self._pack_command(RDT_CMD_START))
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while self._running:
            try:
                data = self.sock.recv(RDT_RECORD_SIZE)
                if len(data) == RDT_RECORD_SIZE:
                    ft = self._unpack_record(data)
                    with self._lock:
                        self._ft = ft
            except socket.timeout:
                continue

    def get_ft(self) -> np.ndarray:
        """Get latest [Fx,Fy,Fz,Tx,Ty,Tz] reading."""
        with self._lock:
            return self._ft.copy()

    def bias(self):
        """Zero the sensor (hardware bias)."""
        self.sock.send(self._pack_command(RDT_CMD_BIAS, 255))
        time.sleep(0.5)

    def stop(self):
        """Stop streaming."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        try:
            self.sock.send(self._pack_command(RDT_CMD_STOP))
        except OSError:
            pass

    def close(self):
        self.stop()
        self.sock.close()


class SdkAdmittanceNode(Node):
    """Admittance control using direct xArm SDK + OnRobot UDP."""

    def __init__(self):
        super().__init__('sdk_admittance')

        # Parameters
        self.declare_parameter('robot_ip', '192.168.1.212')
        self.declare_parameter('ft_sensor_ip', '192.168.2.1')
        self.declare_parameter('rate', 100.0)  # control loop Hz

        # Force-to-velocity gain: v = gain * F
        # Units: (m/s) per N for linear, (rad/s) per Nm for angular
        # Higher = more responsive. Start conservative.
        self.declare_parameter('linear_gain', 0.005)   # 5mm/s per 1N
        self.declare_parameter('angular_gain', 0.01)    # 0.01 rad/s per 1Nm

        # Dead zone: ignore forces below this threshold (noise rejection)
        self.declare_parameter('force_deadzone', 1.0)   # N
        self.declare_parameter('torque_deadzone', 0.1)   # Nm

        # Safety limits
        self.declare_parameter('max_linear_vel', 0.05)   # m/s
        self.declare_parameter('max_angular_vel', 0.2)    # rad/s
        self.declare_parameter('max_displacement', 0.50)  # m from start pose

        # Compliant axes (1=compliant, 0=rigid) in tool frame
        # [x, y, z, rx, ry, rz]
        self.declare_parameter('compliant_axes', [1, 1, 1, 0, 0, 0])

        # Whether to use tool frame (True) or base frame (False) for compliance
        self.declare_parameter('use_tool_frame', True)

        # Bias F/T sensor on startup
        self.declare_parameter('bias_on_start', True)

        self._robot_ip = self.get_parameter('robot_ip').value
        self._ft_ip = self.get_parameter('ft_sensor_ip').value
        self._rate = self.get_parameter('rate').value

        self._arm = None
        self._ft_reader = None
        self._running = False
        self._start_pose = None  # [x,y,z,rx,ry,rz] in mm/rad at activation

        # Service client for controller switching
        self._switch_ctrl_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller'
        )

    def _switch_controllers(self, activate=None, deactivate=None):
        """Switch controllers via controller_manager service."""
        if not self._switch_ctrl_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                'controller_manager/switch_controller service not available. '
                'Continuing anyway (controllers may need manual switching).'
            )
            return False

        req = SwitchController.Request()
        req.activate_controllers = activate or []
        req.deactivate_controllers = deactivate or []
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        future = self._switch_ctrl_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            if future.result().ok:
                self.get_logger().info(
                    f'Controllers switched: +{activate or []} -{deactivate or []}'
                )
                return True
            else:
                self.get_logger().warn('Controller switch returned ok=False')
                return False
        else:
            self.get_logger().warn('Controller switch timed out')
            return False

    def activate(self):
        """Set up SDK connection and start the admittance loop."""
        linear_gain = self.get_parameter('linear_gain').value
        angular_gain = self.get_parameter('angular_gain').value
        self.get_logger().info(
            f'Starting SDK admittance: robot={self._robot_ip}, '
            f'ft_sensor={self._ft_ip}, rate={self._rate}Hz, '
            f'linear_gain={linear_gain}, angular_gain={angular_gain}'
        )

        # Step 1: Connect to OnRobot F/T sensor
        self.get_logger().info('Connecting to OnRobot F/T sensor...')
        try:
            self._ft_reader = OnRobotFTReader(self._ft_ip)
            self._ft_reader.start()
            self.get_logger().info('F/T sensor streaming started.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to F/T sensor: {e}')
            return False

        # Bias if requested
        if self.get_parameter('bias_on_start').value:
            self.get_logger().info('Biasing F/T sensor (zeroing)...')
            self._ft_reader.bias()
            self.get_logger().info('Bias complete.')

        # Step 2: Deactivate position-commanding controllers
        # (keeps joint_state_broadcaster and ft_broadcaster alive)
        self.get_logger().info('Deactivating position controllers...')
        self._switch_controllers(
            deactivate=['xarm6_traj_controller', 'admittance_controller']
        )

        # Step 3: Connect to xArm via Python SDK
        self.get_logger().info(f'Connecting to xArm at {self._robot_ip}...')
        try:
            self._arm = XArmAPI(self._robot_ip)
            time.sleep(0.5)
            if self._arm.error_code != 0:
                self._arm.clean_error()
            if self._arm.warn_code != 0:
                self._arm.clean_warn()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to xArm: {e}')
            return False

        self._arm.motion_enable(True)
        self._arm.set_mode(XARM_MODE_SERVO)
        self._arm.set_state(XARM_STATE_START)
        time.sleep(0.5)

        # Record starting pose for displacement safety check
        code, pose = self._arm.get_position()
        if code == 0:
            self._start_pose = np.array(pose)  # [x,y,z,rx,ry,rz] mm/deg
            self.get_logger().info(
                f'Start pose: [{pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}] mm, '
                f'[{pose[3]:.1f}, {pose[4]:.1f}, {pose[5]:.1f}] deg'
            )
        else:
            self.get_logger().warn(f'Could not read start pose (code={code})')
            self._start_pose = None

        # Step 4: Start control loop
        self._running = True
        self.get_logger().info(
            'Admittance active! Push the end-effector to move it. Ctrl+C to stop.'
        )
        self._run_control_loop()
        return True

    def _run_control_loop(self):
        """Main admittance control loop — runs until stopped."""
        dt = 1.0 / self._rate
        linear_gain = self.get_parameter('linear_gain').value
        angular_gain = self.get_parameter('angular_gain').value
        force_dz = self.get_parameter('force_deadzone').value
        torque_dz = self.get_parameter('torque_deadzone').value
        max_lin_vel = self.get_parameter('max_linear_vel').value
        max_ang_vel = self.get_parameter('max_angular_vel').value
        max_disp = self.get_parameter('max_displacement').value
        compliant = np.array(self.get_parameter('compliant_axes').value, dtype=float)
        use_tool = self.get_parameter('use_tool_frame').value

        cycle_count = 0
        total_displacement = np.zeros(3)  # track cumulative displacement in mm

        while self._running and rclpy.ok():
            loop_start = time.monotonic()

            # 1. Read F/T
            ft = self._ft_reader.get_ft()  # [Fx,Fy,Fz,Tx,Ty,Tz] in N/Nm

            # 2. Apply dead zone
            for i in range(3):
                if abs(ft[i]) < force_dz:
                    ft[i] = 0.0
            for i in range(3, 6):
                if abs(ft[i]) < torque_dz:
                    ft[i] = 0.0

            # 3. Compute velocity: v = gain * F
            vel = np.zeros(6)
            vel[0:3] = linear_gain * ft[0:3]   # m/s
            vel[3:6] = angular_gain * ft[3:6]   # rad/s

            # 4. Apply compliant axes mask
            vel *= compliant

            # 5. Clamp velocity for safety
            lin_speed = np.linalg.norm(vel[0:3])
            if lin_speed > max_lin_vel:
                vel[0:3] *= max_lin_vel / lin_speed
            ang_speed = np.linalg.norm(vel[3:6])
            if ang_speed > max_ang_vel:
                vel[3:6] *= max_ang_vel / ang_speed

            # 6. Compute position delta
            # Linear: mm (xArm API uses mm), Angular: degrees (xArm default)
            dx = np.zeros(6)
            dx[0:3] = vel[0:3] * dt * 1000.0  # m/s → mm per cycle
            dx[3:6] = np.degrees(vel[3:6] * dt)  # rad/s → deg per cycle

            # 7. Safety: check cumulative displacement from start
            total_displacement += dx[0:3]
            disp_m = np.linalg.norm(total_displacement) / 1000.0
            if disp_m > max_disp:
                self.get_logger().warn(
                    f'Max displacement reached ({disp_m:.3f}m > {max_disp}m). '
                    f'Stopping admittance.'
                )
                break

            # 8. Send servo command (only if there's meaningful motion)
            if np.any(np.abs(dx) > 1e-6):
                ret = self._arm.set_servo_cartesian_aa(
                    dx.tolist(), speed=0, mvacc=0,
                    is_tool_coord=use_tool, relative=True
                )
                if ret != 0:
                    self.get_logger().warn(f'set_servo_cartesian_aa ret={ret}')
                    if self._arm.error_code != 0:
                        self.get_logger().error(
                            f'xArm error: {self._arm.error_code}. Stopping.'
                        )
                        break

            # 9. Log periodically
            cycle_count += 1
            if cycle_count % int(self._rate * 2) == 0:  # every 2 seconds
                self.get_logger().info(
                    f'F/T: [{ft[0]:+6.2f}, {ft[1]:+6.2f}, {ft[2]:+6.2f}] N  '
                    f'Disp: {disp_m*100:.1f} cm'
                )

            # 10. Sleep remainder of cycle
            elapsed = time.monotonic() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def deactivate(self):
        """Stop admittance and restore normal operation."""
        self._running = False
        self.get_logger().info('Stopping admittance control...')

        # Disconnect xArm SDK
        if self._arm:
            try:
                # Ensure arm is still in servo mode for HW plugin to take over
                self._arm.set_mode(XARM_MODE_SERVO)
                self._arm.set_state(XARM_STATE_START)
                time.sleep(0.3)
                self._arm.disconnect()
            except Exception:
                pass
            self._arm = None

        # Stop F/T sensor streaming
        if self._ft_reader:
            self._ft_reader.close()
            self._ft_reader = None

        # Reactivate trajectory controller
        self.get_logger().info('Reactivating xarm6_traj_controller...')
        self._switch_controllers(activate=['xarm6_traj_controller'])

        self.get_logger().info('Normal operation restored.')


def main():
    rclpy.init()
    node = SdkAdmittanceNode()

    try:
        node.activate()
    except KeyboardInterrupt:
        pass
    finally:
        node.deactivate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
