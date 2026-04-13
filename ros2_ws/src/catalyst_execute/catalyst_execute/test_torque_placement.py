#!/usr/bin/env python3
"""Test script for well-plate placement with F/T data logging.

Full pick-and-place flow with F/T data logging during admittance placement.

Flow:
  1. Home → close gripper
  2. Approach pick → pick (with guide mode) → retreat → home
  3. Approach place → pre-place
  4. Start F/T logging + admittance placement (same as pick_and_place.py)
  5. Cleanup and recover controllers

Saves F/T log CSV in catalyst_execute/logs/ for analysis.

Usage:
  ros2 run catalyst_execute test_torque_placement --ros-args \
    -p robot_ip:=192.168.1.212 -p ft_sensor_ip:=192.168.2.1
"""

import json
import os
import subprocess
import tempfile
import time
from datetime import datetime

import numpy as np
import yaml
import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from ament_index_python.packages import get_package_share_directory
from catalyst_interfaces.srv import JsonCommand
from xarm.wrapper import XArmAPI

from catalyst_execute.sdk_admittance import OnRobotFTReader

JOINT_SERVICE = '/joint_command'
CARTESIAN_SERVICE = '/cartesian_command'
GRIPPER_SERVICE = '/gripper_command'
GUIDE_MODE_SERVICE = '/guide_mode'

XARM_MODE_CART_VELOCITY = 5
XARM_STATE_START = 0

PRE_HEIGHT = 0.05  # 5cm above pick/place


def quat_to_rotation_matrix(qx, qy, qz, qw):
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


def rotation_matrix_to_quat(R):
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def pose_to_homogeneous(position, orientation):
    H = np.eye(4)
    H[:3, :3] = quat_to_rotation_matrix(
        orientation['qx'], orientation['qy'],
        orientation['qz'], orientation['qw']
    )
    H[0, 3] = position['x']
    H[1, 3] = position['y']
    H[2, 3] = position['z']
    return H


def homogeneous_to_pos_quat(H):
    pos = H[:3, 3]
    qx, qy, qz, qw = rotation_matrix_to_quat(H[:3, :3])
    return pos[0], pos[1], pos[2], qx, qy, qz, qw


class TestTorquePlacementNode(Node):
    def __init__(self):
        super().__init__('test_torque_placement')

        # Load calibrated transforms
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'gripper_pose.json'
        )
        if not os.path.exists(config_path):
            config_path = os.path.join(
                get_package_share_directory('catalyst_execute'),
                'config', 'gripper_pose.json'
            )
        with open(config_path, 'r') as f:
            config = json.load(f)
        self._H_TCP_stand1 = np.array(config['H_TCP_TO_stand1'])
        self._H_TCP_stand2 = np.array(config['H_TCP_TO_stand2'])
        self.get_logger().info(f'Loaded transforms from {config_path}')

        # World model
        self._world_model = None
        self._world_model_ts = None
        self.create_subscription(String, '/world_model', self._world_model_cb, 10)

        # Parameters
        self.declare_parameter('robot_ip', '192.168.1.212')
        self.declare_parameter('ft_sensor_ip', '192.168.2.1')
        self.declare_parameter('rate', 100.0)

        # Admittance parameters
        self.declare_parameter('linear_gain', 0.005)
        self.declare_parameter('force_deadzone', 1.0)
        self.declare_parameter('ref_velocity_x', -0.005)
        self.declare_parameter('place_force_threshold', 7.0)
        self.declare_parameter('max_place_distance', 0.1)
        self.declare_parameter('compliant_axes', [0, 1, 1, 0, 0, 0])
        self.declare_parameter('max_linear_vel', 0.05)

        # Corner registration parameters
        self.declare_parameter('contact_force_threshold', 1.0)  # |Fx| to detect plate touching stand
        self.declare_parameter('corner_force_threshold', 1.0)   # |Fy|/|Fz| to detect corner contact
        self.declare_parameter('search_speed', 0.003)           # m/s — lateral search speed

        # Service clients
        self._joint_client = self.create_client(JsonCommand, JOINT_SERVICE)
        self._cartesian_client = self.create_client(JsonCommand, CARTESIAN_SERVICE)
        self._gripper_client = self.create_client(JsonCommand, GRIPPER_SERVICE)
        self._guide_mode_client = self.create_client(JsonCommand, GUIDE_MODE_SERVICE)
        self._clear_octomap_client = self.create_client(Empty, '/clear_octomap')
        self._set_octomap_client = self.create_client(SetBool, '/set_octomap_enabled')

        self._arm = None
        self._ft_reader = None

    def _world_model_cb(self, msg: String):
        data = json.loads(msg.data)
        self._world_model_ts = data.get('timestamp')
        self._world_model = data

    def wait_for_services(self, timeout=None):
        self.get_logger().info('Waiting for services...')
        self._joint_client.wait_for_service(timeout_sec=timeout)
        self._cartesian_client.wait_for_service(timeout_sec=timeout)
        self._gripper_client.wait_for_service(timeout_sec=timeout)
        self._guide_mode_client.wait_for_service(timeout_sec=timeout)
        self.get_logger().info('All services ready.')

    def wait_for_world_model(self, timeout=10.0):
        self.get_logger().info('Waiting for world model...')
        start = time.time()
        while self._world_model is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('Timed out waiting for /world_model')
                return False
        return True

    # ── Service helpers ──

    def _call(self, client, command):
        req = JsonCommand.Request()
        req.command = json.dumps(command)
        self.get_logger().info(f'Sending: {req.command}')
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        if future.result() is not None:
            resp = json.loads(future.result().response)
            self.get_logger().info(f'Result: {resp["message"]}')
            return resp['success']
        self.get_logger().error('Service call failed')
        return False

    def move_pose(self, pose_name, speed=0.1):
        return self._call(self._joint_client, {'pose': pose_name, 'speed': speed})

    def move_cartesian(self, x, y, z, qx, qy, qz, qw, speed=0.1,
                       keep_orientation=False, straight_line=False):
        return self._call(self._cartesian_client, {
            'x': x, 'y': y, 'z': z,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'speed': speed,
            'keep_orientation': keep_orientation,
            'straight_line': straight_line,
        })

    def open_gripper(self):
        return self._call(self._gripper_client, {'action': 'open'})

    def close_gripper(self):
        return self._call(self._gripper_client, {'action': 'close'})

    def activate_guide_mode(self, mode):
        return self._call(self._guide_mode_client, {'action': mode, 'sensitivity': 5})

    def clear_octomap(self):
        future = self._clear_octomap_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def set_octomap_enabled(self, enabled: bool):
        req = SetBool.Request()
        req.data = enabled
        future = self._set_octomap_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def get_current_tcp_pose(self):
        rclpy.spin_once(self, timeout_sec=0.5)
        if self._world_model is None:
            return None
        tcp = self._world_model.get('tcp_pose')
        if tcp is None:
            return None
        p = tcp['position']
        o = tcp['orientation']
        return [p['x'], p['y'], p['z'], o['qx'], o['qy'], o['qz'], o['qw']]

    # ── Pose computation ──

    def compute_pose_from_tag(self, tag_pose, H_TCP_TO):
        H_tag = pose_to_homogeneous(tag_pose['position'], tag_pose['orientation'])
        return H_tag @ H_TCP_TO

    def compute_approach_pose(self, H_target, H_tag):
        tag_pos = H_tag[:3, 3]
        tag_z = H_tag[:3, 2]
        target_pos = H_target[:3, 3]
        depth = np.dot(target_pos - tag_pos, tag_z)
        offset = 0.05 - depth
        approach_pos = target_pos + offset * tag_z
        H_approach = H_target.copy()
        H_approach[:3, 3] = approach_pos
        return H_approach

    # ── Pick sequence ──

    def execute_pick(self, H_pick):
        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_pick)

        self.get_logger().info('=== Moving to pre-pick ===')
        if not self.move_cartesian(x, y, z + PRE_HEIGHT, qx, qy, qz, qw):
            return False
        time.sleep(0.5)

        self.get_logger().info('=== Opening gripper ===')
        self.open_gripper()
        time.sleep(0.5)

        self.set_octomap_enabled(False)

        self.get_logger().info('=== Moving to pick pose ===')
        if not self.move_cartesian(x, y, z, qx, qy, qz, qw, speed=0.01):
            return False
        time.sleep(0.5)

        self.get_logger().info('=== Closing gripper ===')
        self.close_gripper()
        time.sleep(1.0)

        # Guide mode for user to verify grasp
        self.get_logger().info('=== Guide mode enabled — press Enter to continue ===')
        self.activate_guide_mode('enable')
        self.activate_guide_mode('disable')
        time.sleep(0.5)

        current_tcp = self.get_current_tcp_pose()
        if current_tcp is not None:
            cx, cy, cz, cqx, cqy, cqz, cqw = current_tcp
            self.get_logger().info('=== Retracting from current pose ===')
            self.move_cartesian(cx, cy, cz + PRE_HEIGHT, cqx, cqy, cqz, cqw,
                                speed=0.01, keep_orientation=True, straight_line=True)
        else:
            self.get_logger().info('=== Retracting to pre-pick ===')
            self.move_cartesian(x, y, z + PRE_HEIGHT, qx, qy, qz, qw,
                                speed=0.01, keep_orientation=True, straight_line=True)
        time.sleep(0.5)
        self.set_octomap_enabled(True)
        return True

    # ── Corner registration placement with F/T logging ──

    def run_admittance_place(self, log_path):
        """Descend until contact, then search left (Y) then forward (Z) to find corner."""
        robot_ip = self.get_parameter('robot_ip').value
        ft_ip = self.get_parameter('ft_sensor_ip').value
        rate = self.get_parameter('rate').value
        dt = 1.0 / rate

        ref_vx = self.get_parameter('ref_velocity_x').value
        max_dist = self.get_parameter('max_place_distance').value
        contact_force = self.get_parameter('contact_force_threshold').value
        corner_force = self.get_parameter('corner_force_threshold').value
        search_speed = self.get_parameter('search_speed').value

        # Connect to F/T sensor
        self.get_logger().info(f'Connecting to F/T sensor at {ft_ip}...')
        self._ft_reader = OnRobotFTReader(ft_ip)
        self._ft_reader.start()
        self._ft_reader.bias()
        self.get_logger().info('F/T sensor biased.')

        # Connect to xArm SDK
        self.get_logger().info(f'Connecting to xArm at {robot_ip}...')
        self._arm = XArmAPI(robot_ip, protocol=3)
        time.sleep(0.5)
        if self._arm.error_code != 0:
            self._arm.clean_error()
        if self._arm.warn_code != 0:
            self._arm.clean_warn()
        self._arm.motion_enable(True)
        self._arm.set_collision_sensitivity(0)
        self._arm.set_mode(XARM_MODE_CART_VELOCITY)
        self._arm.set_state(XARM_STATE_START)
        time.sleep(0.5)
        self.get_logger().info(f'Arm mode: {self._arm.mode} (expected {XARM_MODE_CART_VELOCITY})')

        code, start_pose = self._arm.get_position()
        if code != 0:
            self.get_logger().error(f'Cannot read arm position (code={code})')
            return False, 'Failed to read arm position'

        # Phase state machine: DESCEND → SEARCH_Z → SEARCH_Y → DONE
        phase = 'DESCEND'
        corner_pose = None

        self.get_logger().info(
            f'Corner registration: descend at {ref_vx*1000:.1f} mm/s, '
            f'contact at {contact_force:.1f} N, corner at {corner_force:.1f} N, '
            f'search speed {search_speed*1000:.1f} mm/s'
        )

        cycle_count = 0
        reason = 'Interrupted'

        log_file = open(log_path, 'w')
        log_file.write('time,fx,fy,fz,tx,ty,tz,vx,vy,vz,travel_mm,pos_x,pos_y,pos_z,phase\n')

        t_start = time.monotonic()

        while rclpy.ok():
            loop_start = time.monotonic()
            t_elapsed = time.monotonic() - t_start

            ft = self._ft_reader.get_ft()

            vel_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            if phase == 'DESCEND':
                # Descend in -X (tool frame) until contact
                vel_cmd[0] = ref_vx * 1000  # mm/s
                if abs(ft[0]) > contact_force:
                    self.get_logger().info(
                        f'Contact detected! |Fx|={abs(ft[0]):.2f} N > {contact_force} N'
                    )
                    phase = 'SEARCH_Z'
                    self.get_logger().info('Searching forward (Z) for corner...')

            elif phase == 'SEARCH_Z':
                # Move forward in +Z (tool frame)
                vel_cmd[2] = search_speed * 1000  # mm/s
                if abs(ft[2]) > corner_force:
                    self.get_logger().info(
                        f'Corner Z found! |Fz|={abs(ft[2]):.2f} N > {corner_force} N'
                    )
                    phase = 'SEARCH_Y'
                    self.get_logger().info('Searching left (Y) for corner...')

            elif phase == 'SEARCH_Y':
                vel_cmd[1] = -search_speed * 1000  # mm/s
                if abs(ft[1]) > corner_force:
                    code_c, pose_c = self._arm.get_position()
                    if code_c == 0:
                        corner_pose = pose_c
                    self.get_logger().info(
                        f'Corner Y found! |Fy|={abs(ft[1]):.2f} N > {corner_force} N'
                    )
                    self.get_logger().info(
                        f'Corner position (mm): '
                        f'X={corner_pose[0]:.2f}, Y={corner_pose[1]:.2f}, Z={corner_pose[2]:.2f}'
                        if corner_pose else 'Failed to read position'
                    )
                    phase = 'DONE'
                    reason = 'Corner found'
                    break

            # Check travel distance
            code, actual_pose = self._arm.get_position()
            actual_travel = 0.0
            if code == 0:
                actual_travel = np.linalg.norm(
                    np.array(actual_pose[0:3]) - np.array(start_pose[0:3])
                ) / 1000.0
                if actual_travel > max_dist:
                    reason = f'Max distance: {actual_travel*1000:.1f} mm'
                    break

            # Send velocity
            ret = self._arm.vc_set_cartesian_velocity(vel_cmd, is_tool_coord=True, duration=-1)
            if ret != 0:
                self.get_logger().warn(f'vc_set ret={ret}')
                if self._arm.error_code != 0:
                    reason = f'xArm error: {self._arm.error_code}'
                    break

            # Log F/T data
            raw_ft = self._ft_reader.get_ft()
            px = actual_pose[0] if code == 0 else 0.0
            py = actual_pose[1] if code == 0 else 0.0
            pz = actual_pose[2] if code == 0 else 0.0
            log_file.write(
                f'{t_elapsed:.4f},'
                f'{raw_ft[0]:.4f},{raw_ft[1]:.4f},{raw_ft[2]:.4f},'
                f'{raw_ft[3]:.5f},{raw_ft[4]:.5f},{raw_ft[5]:.5f},'
                f'{vel_cmd[0]:.2f},{vel_cmd[1]:.2f},{vel_cmd[2]:.2f},'
                f'{actual_travel*1000:.1f},'
                f'{px:.2f},{py:.2f},{pz:.2f},'
                f'{phase}\n'
            )
            log_file.flush()

            cycle_count += 1
            if cycle_count % int(rate) == 0:
                self.get_logger().info(
                    f'[{phase}] F:[{raw_ft[0]:+6.2f},{raw_ft[1]:+6.2f},{raw_ft[2]:+6.2f}] N  '
                    f'T:[{raw_ft[3]:+6.3f},{raw_ft[4]:+6.3f},{raw_ft[5]:+6.3f}] Nm  '
                    f'Pos:({px:.1f},{py:.1f},{pz:.1f})'
                )

            elapsed = time.monotonic() - loop_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

        self._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_tool_coord=True)
        log_file.close()
        self.get_logger().info(f'Corner registration stopped: {reason}')
        self.get_logger().info(f'F/T log saved to {log_path}')
        return corner_pose is not None, reason, corner_pose

    def cleanup_admittance(self):
        if self._ft_reader:
            self._ft_reader.close()
            self._ft_reader = None
        if self._arm:
            try:
                self._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_tool_coord=True)
                time.sleep(0.3)
                self._arm.set_state(4)
                time.sleep(0.5)
                self._arm.set_collision_sensitivity(3)
                self.get_logger().info(
                    f'SDK stopped: mode={self._arm.mode}, state={self._arm.state}'
                )
                self._arm.disconnect()
                self.get_logger().info('SDK disconnected.')
            except Exception as e:
                self.get_logger().warn(f'Cleanup error: {e}')
            self._arm = None

    def restart_ros2_control(self):
        """Kill ros2_control_node, restart it, re-spawn controllers."""
        self.get_logger().info('=== Restarting ros2_control_node ===')

        # 1. Get robot_description from robot_state_publisher
        self.get_logger().info('Fetching robot_description...')
        param_client = self.create_client(
            GetParameters, '/robot_state_publisher/get_parameters'
        )
        if not param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Cannot reach robot_state_publisher')
            return False

        req = GetParameters.Request()
        req.names = ['robot_description']
        future = param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            self.get_logger().error('Failed to get robot_description')
            return False
        urdf = future.result().values[0].string_value

        # 2. Get controller config path
        bringup_pkg = get_package_share_directory('catalyst_bringup')
        controller_config = os.path.join(bringup_pkg, 'config', 'ros2_controllers_real.yaml')

        # 3. Write URDF to temp param file
        param_data = {
            'ros2_control_node': {
                'ros__parameters': {
                    'robot_description': urdf
                }
            }
        }
        param_file = tempfile.NamedTemporaryFile(
            mode='w', suffix='.yaml', delete=False, prefix='ros2ctrl_'
        )
        yaml.dump(param_data, param_file, default_flow_style=False)
        param_file.close()

        # 4. Kill ros2_control_node
        self.get_logger().info('Killing ros2_control_node...')
        subprocess.run(['pkill', '-9', '-f', 'ros2_control_node'], timeout=5)
        time.sleep(3)

        # 5. Start new ros2_control_node
        self.get_logger().info('Starting new ros2_control_node...')
        subprocess.Popen(
            [
                'ros2', 'run', 'controller_manager', 'ros2_control_node',
                '--ros-args',
                '--params-file', param_file.name,
                '--params-file', controller_config,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(3)

        # 6. Spawn controllers
        controllers = [
            'joint_state_broadcaster',
            'xarm6_traj_controller',
            'force_torque_sensor_broadcaster',
        ]
        for ctrl in controllers:
            self.get_logger().info(f'Spawning {ctrl}...')
            result = subprocess.run(
                ['ros2', 'run', 'controller_manager', 'spawner',
                 ctrl, '-c', '/controller_manager'],
                timeout=30, capture_output=True, text=True,
            )
            if result.returncode != 0:
                self.get_logger().error(f'{ctrl} spawn failed: {result.stderr}')
                if ctrl != 'force_torque_sensor_broadcaster':
                    return False

        self.get_logger().info('ros2_control_node restarted and controllers spawned.')
        time.sleep(2)

        try:
            os.unlink(param_file.name)
        except OSError:
            pass

        return True


def main():
    rclpy.init()
    node = TestTorquePlacementNode()

    try:
        node.wait_for_services()

        if not node.wait_for_world_model():
            node.get_logger().error('No world model — aborting')
            return

        # Hardcoded tag pose (testing)
        tag_pose = {
            'position': {'x': -0.321512, 'y': 0.590565, 'z': 0.079846},
            'orientation': {'qx': -0.039334, 'qy': 0.706857, 'qz': -0.706159, 'qw': -0.012113},
        }

        # Compute poses
        H_pick = node.compute_pose_from_tag(tag_pose, node._H_TCP_stand1)
        H_place = node.compute_pose_from_tag(tag_pose, node._H_TCP_stand2)
        H_tag = pose_to_homogeneous(tag_pose['position'], tag_pose['orientation'])
        H_approach_pick = node.compute_approach_pose(H_pick, H_tag)
        H_approach_place = node.compute_approach_pose(H_place, H_tag)

        px, py, pz, pqx, pqy, pqz, pqw = homogeneous_to_pos_quat(H_pick)
        lx, ly, lz, lqx, lqy, lqz, lqw = homogeneous_to_pos_quat(H_place)
        apx, apy, apz, apqx, apqy, apqz, apqw = homogeneous_to_pos_quat(H_approach_pick)
        alx, aly, alz, alqx, alqy, alqz, alqw = homogeneous_to_pos_quat(H_approach_place)

        node.get_logger().info(f'Pick:  ({px:.4f}, {py:.4f}, {pz:.4f})')
        node.get_logger().info(f'Place: ({lx:.4f}, {ly:.4f}, {lz:.4f})')

        # 1. Home
        node.get_logger().info('========== Home ==========')
        node.move_pose('home')
        node.clear_octomap()
        time.sleep(0.5)
        node.close_gripper()
        time.sleep(0.5)

        # 2. Approach pick
        node.set_octomap_enabled(True)
        node.get_logger().info('========== Approach pick ==========')
        if not node.move_cartesian(apx, apy, apz + PRE_HEIGHT, apqx, apqy, apqz, apqw):
            node.get_logger().error('Failed to reach pick approach')
            return
        time.sleep(0.5)

        # 3. Pick
        node.get_logger().info('========== Pick ==========')
        if not node.execute_pick(H_pick):
            node.get_logger().error('Pick failed')
            return

        # 4. Retreat from pick → home
        node.get_logger().info('========== Retreat pick ==========')
        node.set_octomap_enabled(False)
        node.move_cartesian(apx, apy, apz, apqx, apqy, apqz, apqw, keep_orientation=True)
        time.sleep(0.5)

        node.get_logger().info('========== Home ==========')
        node.move_pose('home')
        time.sleep(0.5)

        # 5. Approach place — use offset position to intentionally hit a corner
        # Offset x,y from TCP pose that lands near one corner of the stand
        offset_x = 0.117472
        offset_y = 0.778676
        node.get_logger().info(
            f'Using offset pre-place: x={offset_x:.4f}, y={offset_y:.4f} '
            f'(normal: x={lx:.4f}, y={ly:.4f})'
        )

        node.set_octomap_enabled(True)
        node.get_logger().info('========== Approach place (offset) ==========')
        if not node.move_cartesian(offset_x, offset_y, lz + PRE_HEIGHT,
                                   lqx, lqy, lqz, lqw):
            node.get_logger().error('Failed to reach place approach')
            return
        time.sleep(0.5)

        # 6. Pre-place at offset position
        node.get_logger().info('========== Pre-place (offset) ==========')
        if not node.move_cartesian(offset_x, offset_y, lz + PRE_HEIGHT,
                                   lqx, lqy, lqz, lqw):
            node.get_logger().error('Failed to reach pre-place')
            return
        time.sleep(1.0)

        node.set_octomap_enabled(False)

        # 7. Admittance placement with F/T logging
        log_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'logs'
        )
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(log_dir, f'torque_placement_{timestamp}.csv')

        node.get_logger().info('========== Corner registration ==========')
        success, reason, corner_pose = node.run_admittance_place(log_path)

        if success and corner_pose is not None:
            node.get_logger().info(f'Corner found: {reason}')

            # Switch SDK from velocity mode (5) to position mode (0)
            node._arm.set_state(4)  # STOP
            time.sleep(0.5)
            node._arm.set_mode(0)   # Position mode
            node._arm.set_state(0)  # START
            time.sleep(0.5)
            node.get_logger().info(
                f'SDK position mode: mode={node._arm.mode}, state={node._arm.state}'
            )

            # Read current position from SDK (mm, degrees)
            code, current_pose = node._arm.get_position()
            if code != 0:
                node.get_logger().error('Cannot read current position')
                node.cleanup_admittance()
                return
            cx, cy, cz = current_pose[0], current_pose[1], current_pose[2]
            node.get_logger().info(f'Current (mm): X={cx:.2f}, Y={cy:.2f}, Z={cz:.2f}')

            # Corrected pose: lift 5mm, shift +2mm X, -2mm Y from current
            corrected_x = cx - 5.0   # mm
            corrected_y = cy + 5.0   # mm
            corrected_z = cz + 5.0   # mm

            node.get_logger().info(
                f'Corrected (mm): X={corrected_x:.2f}, Y={corrected_y:.2f}, Z={corrected_z:.2f}'
            )

            # Move to corrected pose (lift + shift) — slow speed
            node.get_logger().info('========== Moving to corrected pose ==========')
            node._arm.set_position(
                x=corrected_x, y=corrected_y, z=corrected_z,
                roll=current_pose[3], pitch=current_pose[4], yaw=current_pose[5],
                speed=5, wait=True
            )
            time.sleep(0.3)

            # Verify X/Y are within 0.5mm, keep correcting if not
            xy_tol = 0.5  # mm
            max_corrections = 10
            for i in range(max_corrections):
                code_v, verify_pose = node._arm.get_position()
                if code_v != 0:
                    break
                dx = corrected_x - verify_pose[0]
                dy = corrected_y - verify_pose[1]
                node.get_logger().info(
                    f'Correction check {i+1}: dx={dx:+.2f} mm, dy={dy:+.2f} mm'
                )
                if abs(dx) < xy_tol and abs(dy) < xy_tol:
                    node.get_logger().info('Position within tolerance.')
                    break
                node._arm.set_position(
                    x=corrected_x, y=corrected_y, z=corrected_z,
                    roll=current_pose[3], pitch=current_pose[4], yaw=current_pose[5],
                    speed=5, wait=True
                )
                time.sleep(0.3)

            # Descend using velocity mode with F/T force stop
            node.get_logger().info('========== Descending with force control ==========')
            node._arm.set_state(4)  # STOP
            time.sleep(0.3)
            node._arm.set_mode(XARM_MODE_CART_VELOCITY)
            node._arm.set_state(XARM_STATE_START)
            time.sleep(0.3)

            # Re-bias F/T sensor before descent
            node._ft_reader.bias()
            time.sleep(0.2)

            descent_speed = -5.0  # mm/s (tool -X)
            place_force = 5.0  # N
            while rclpy.ok():
                ft = node._ft_reader.get_ft()
                if abs(ft[0]) > place_force:
                    node.get_logger().info(
                        f'Plate seated! |Fx|={abs(ft[0]):.2f} N > {place_force} N'
                    )
                    break
                node._arm.vc_set_cartesian_velocity(
                    [descent_speed, 0, 0, 0, 0, 0], is_tool_coord=True, duration=-1
                )
                time.sleep(0.01)

            node._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_tool_coord=True)
            time.sleep(0.5)
        else:
            node.get_logger().warn(f'Corner registration INCOMPLETE: {reason}')

        node.cleanup_admittance()

        # 8. Open gripper
        node.get_logger().info('========== Release plate ==========')
        # node.open_gripper()
        time.sleep(0.5)

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
        node.cleanup_admittance()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
