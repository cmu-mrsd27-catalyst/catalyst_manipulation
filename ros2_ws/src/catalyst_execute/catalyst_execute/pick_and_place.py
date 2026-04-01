#!/usr/bin/env python3
"""Full pick-and-place pipeline using AprilTag detection and admittance placement.

Flow:
  1. Home
  2. Explore for AprilTag
  3. Compute pick (stand 1) and place (stand 2) poses from tag
  4. Open gripper → pre-pick → pick → close gripper
  5. Retract to pre-pick → home
  6. Pre-place → admittance-controlled placement
  7. Open gripper → retract to pre-place → home

Usage:
  ros2 run catalyst_execute pick_and_place --ros-args \
    -p robot_ip:=192.168.1.212 -p ft_sensor_ip:=192.168.2.1
"""

import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Empty, SetBool
from ament_index_python.packages import get_package_share_directory
from catalyst_interfaces.srv import JsonCommand
from xarm.wrapper import XArmAPI

from catalyst_execute.sdk_admittance import OnRobotFTReader
from catalyst_execute.exploration import TagExplorer

JOINT_SERVICE = '/joint_command'
CARTESIAN_SERVICE = '/cartesian_command'
GRIPPER_SERVICE = '/gripper_command'

# xArm mode/state constants
XARM_MODE_SERVO = 1
XARM_MODE_CART_VELOCITY = 5
XARM_STATE_START = 0

PRE_HEIGHT = 0.05  # 5cm above pick/place pose


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


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place')

        # Load calibrated transforms from gripper_pose.json
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
        self.get_logger().info(f'Loaded pick/place transforms from {config_path}')

        # World model subscription
        self._world_model = None
        self._world_model_ts = None
        self.create_subscription(String, '/world_model', self._world_model_cb, 10)

        # Parameters
        self.declare_parameter('robot_ip', '192.168.1.212')
        self.declare_parameter('ft_sensor_ip', '192.168.2.1')
        self.declare_parameter('rate', 100.0)

        # Admittance parameters
        # Push axis is tool X (gripper rotated -90° pitch, so flange X = downward)
        self.declare_parameter('linear_gain', 0.005)
        self.declare_parameter('force_deadzone', 1.0)
        self.declare_parameter('ref_velocity_x', -0.005)
        self.declare_parameter('place_force_threshold', 7.0)
        self.declare_parameter('max_place_distance', 0.1)
        self.declare_parameter('compliant_axes', [0, 1, 1, 0, 0, 0])
        self.declare_parameter('max_linear_vel', 0.05)

        # Service clients
        self._joint_client = self.create_client(JsonCommand, JOINT_SERVICE)
        self._cartesian_client = self.create_client(JsonCommand, CARTESIAN_SERVICE)
        self._gripper_client = self.create_client(JsonCommand, GRIPPER_SERVICE)
        self._clear_octomap_client = self.create_client(Empty, '/clear_octomap')
        self._set_octomap_client = self.create_client(SetBool, '/set_octomap_enabled')

        self._arm = None
        self._ft_reader = None

    def _world_model_cb(self, msg: String):
        data = json.loads(msg.data)
        self._world_model_ts = data.get('timestamp')
        self._world_model = data

    def get_world_model(self):
        return (self._world_model_ts, self._world_model)

    def wait_for_services(self):
        self.get_logger().info('Waiting for services...')
        self._joint_client.wait_for_service()
        self._cartesian_client.wait_for_service()
        self._gripper_client.wait_for_service()
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

    def move_joints(self, joints_deg, speed=0.1):
        return self._call(self._joint_client, {'joints': joints_deg, 'speed': speed})

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

    def clear_octomap(self):
        future = self._clear_octomap_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    def set_octomap_enabled(self, enabled: bool):
        req = SetBool.Request()
        req.data = enabled
        future = self._set_octomap_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    # ── Exploration ──

    def explore_for_tag(self):
        explorer = TagExplorer(
            node=self,
            joint_client=self._joint_client,
            world_model_getter=self.get_world_model,
        )
        return explorer.search()

    # ── Pose computation ──

    def compute_pose_from_tag(self, tag_pose, H_TCP_TO):
        H_tag = pose_to_homogeneous(tag_pose['position'], tag_pose['orientation'])
        return H_tag @ H_TCP_TO

    # ── Pick sequence (MoveIt) ──

    def execute_pick(self, H_pick, tag_j1):
        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_pick)
        self.get_logger().info(
            f'Pick pose: ({x:.4f}, {y:.4f}, {z:.4f})'
        )

        # Open gripper before picking
        self.get_logger().info('=== Opening gripper ===')
        self.open_gripper()
        time.sleep(0.5)

        # Face the machine
        self.get_logger().info(f'=== Facing machine (j1={tag_j1:.1f} deg) ===')
        self.move_joints([tag_j1, 0.0, 0.0, 0.0, 0.0, 0.0])
        time.sleep(0.5)

        # Pre-pick (5cm above)
        self.get_logger().info('=== Moving to pre-pick ===')
        if not self.move_cartesian(x, y, z + PRE_HEIGHT, qx, qy, qz, qw):
            self.get_logger().error('Failed to reach pre-pick')
            return False
        time.sleep(0.5)

        # Disable octomap for close-range pick motion
        self.set_octomap_enabled(False)

        # Pick
        self.get_logger().info('=== Moving to pick pose ===')
        if not self.move_cartesian(x, y, z, qx, qy, qz, qw,
                                   speed=0.01):
            self.get_logger().error('Failed to reach pick pose')
            self.set_octomap_enabled(True)
            return False
        time.sleep(0.5)

        # Close gripper to grasp
        self.get_logger().info('=== Closing gripper ===')
        self.close_gripper()
        time.sleep(1.0)

        # Re-enable octomap, clear it (grasped plate would be obstacle)
        # self.clear_octomap()

        # Retract to pre-pick
        self.get_logger().info('=== Retracting to pre-pick ===')
        self.move_cartesian(x, y, z + PRE_HEIGHT, qx, qy, qz, qw,
                            speed=0.01, keep_orientation=True, straight_line=True)
        time.sleep(0.5)
        self.set_octomap_enabled(True)

        return True

    # ── Admittance placement ──

    def run_admittance_place(self):
        robot_ip = self.get_parameter('robot_ip').value
        ft_ip = self.get_parameter('ft_sensor_ip').value
        rate = self.get_parameter('rate').value
        dt = 1.0 / rate

        linear_gain = self.get_parameter('linear_gain').value
        force_dz = self.get_parameter('force_deadzone').value
        ref_vx = self.get_parameter('ref_velocity_x').value
        place_force = self.get_parameter('place_force_threshold').value
        max_dist = self.get_parameter('max_place_distance').value
        compliant = np.array(self.get_parameter('compliant_axes').value, dtype=float)
        max_lin_vel = self.get_parameter('max_linear_vel').value

        # Connect to F/T sensor
        self.get_logger().info(f'Connecting to F/T sensor at {ft_ip}...')
        self._ft_reader = OnRobotFTReader(ft_ip)
        self._ft_reader.start()
        self._ft_reader.bias()
        self.get_logger().info('F/T sensor biased.')

        # Connect to xArm SDK, enter cartesian velocity mode
        self.get_logger().info(f'Connecting to xArm at {robot_ip}...')
        self._arm = XArmAPI(robot_ip)
        time.sleep(0.5)
        if self._arm.error_code != 0:
            self._arm.clean_error()
        if self._arm.warn_code != 0:
            self._arm.clean_warn()
        self._arm.motion_enable(True)
        self._arm.set_mode(XARM_MODE_CART_VELOCITY)
        self._arm.set_state(XARM_STATE_START)
        time.sleep(0.5)
        self.get_logger().info(f'Arm mode: {self._arm.mode} (expected {XARM_MODE_CART_VELOCITY})')

        # Record start pose
        code, start_pose = self._arm.get_position()
        if code != 0:
            self.get_logger().error(f'Cannot read arm position (code={code})')
            return False, 'Failed to read arm position'

        self.get_logger().info(
            f'Admittance active! ref_vx={ref_vx*1000:.1f} mm/s, '
            f'stop at {place_force:.1f} N or {max_dist*1000:.0f} mm'
        )

        cycle_count = 0
        place_success = False
        reason = 'Interrupted'

        while rclpy.ok():
            loop_start = time.monotonic()

            ft = self._ft_reader.get_ft()

            # Push axis is now tool X (flange X = downward with rotated gripper)
            if abs(ft[0]) > place_force:
                reason = f'Force threshold: Fx={ft[0]:+.2f} N > {place_force} N'
                place_success = True
                break

            for i in range(3):
                if abs(ft[i]) < force_dz:
                    ft[i] = 0.0

            vel = np.zeros(6)
            vel[0:3] = linear_gain * ft[0:3]
            vel *= compliant
            vel[0] += ref_vx

            lin_speed = np.linalg.norm(vel[0:3])
            if lin_speed > max_lin_vel:
                vel[0:3] *= max_lin_vel / lin_speed

            code, actual_pose = self._arm.get_position()
            if code == 0:
                actual_travel = np.linalg.norm(
                    np.array(actual_pose[0:3]) - np.array(start_pose[0:3])
                ) / 1000.0
                if actual_travel > max_dist:
                    reason = f'Max distance: {actual_travel*1000:.1f} mm > {max_dist*1000:.0f} mm'
                    break

            vel_cmd = [vel[0]*1000, vel[1]*1000, vel[2]*1000, 0, 0, 0]
            ret = self._arm.vc_set_cartesian_velocity(vel_cmd, is_tool_coord=True, duration=-1)
            if ret != 0:
                self.get_logger().warn(f'vc_set ret={ret}')
                if self._arm.error_code != 0:
                    reason = f'xArm error: {self._arm.error_code}'
                    break

            cycle_count += 1
            if cycle_count % int(rate * 2) == 0:
                travel_mm = actual_travel * 1000.0 if code == 0 else -1
                self.get_logger().info(
                    f'F/T: [{ft[0]:+6.2f}, {ft[1]:+6.2f}, {ft[2]:+6.2f}] N  '
                    f'Travel: {travel_mm:.1f} mm'
                )

            elapsed = time.monotonic() - loop_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

        self._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_tool_coord=True)
        self.get_logger().info(f'Admittance stopped: {reason}')
        return place_success, reason

    def cleanup_admittance(self):
        if self._ft_reader:
            self._ft_reader.close()
            self._ft_reader = None

        if self._arm:
            try:
                # 1. Stop velocity output
                self._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_tool_coord=True)
                time.sleep(0.3)
                # 2. CRITICAL: set_state(STOP) on the SAME connection that entered mode 5.
                #    Firmware won't accept mode changes from an active velocity stream.
                self._arm.set_state(4)  # STOP
                time.sleep(0.5)
                self.get_logger().info(
                    f'Velocity stopped: mode={self._arm.mode}, state={self._arm.state}'
                )
                # 3. Now disconnect — firmware is in stopped state, ready for mode change
                self._arm.disconnect()
                self.get_logger().info('Admittance SDK disconnected.')
            except Exception as e:
                self.get_logger().warn(f'SDK cleanup error: {e}')
            self._arm = None

        # Use motion_planner's guide_mode service to restore servo mode.
        # It uses the HW plugin's SDK connection so mode changes are seen properly.
        self.get_logger().info('Restoring servo mode via guide_mode service...')
        guide_client = self.create_client(JsonCommand, '/guide_mode')
        if guide_client.wait_for_service(timeout_sec=5.0):
            req = JsonCommand.Request()
            req.command = json.dumps({'action': 'disable'})
            future = guide_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            if future.result() is not None:
                resp = json.loads(future.result().response)
                self.get_logger().info(f'Guide mode disable: {resp["message"]}')
            else:
                self.get_logger().warn('Guide mode disable call failed')
        else:
            self.get_logger().warn('/guide_mode service not available')


def main():
    rclpy.init()
    node = PickAndPlaceNode()

    try:
        node.wait_for_services()

        if not node.wait_for_world_model():
            node.get_logger().error('No world model — aborting')
            return

        # 1. Home
        node.get_logger().info('========== Moving to home ==========')
        if not node.move_pose('home'):
            node.get_logger().error('Failed to move home')
            return
        
        time.sleep(0.5)
        node.open_gripper()
        time.sleep(0.5)

        # 2. Explore for AprilTag
        node.get_logger().info('========== Exploring for AprilTag ==========')
        tag_pose, tag_j1 = node.explore_for_tag()
        if tag_pose is None:
            node.get_logger().error('Tag not found. Aborting.')
            return

        # 3. Compute pick (stand 1) and place (stand 2) poses
        H_pick = node.compute_pose_from_tag(tag_pose, node._H_TCP_stand1)
        H_place = node.compute_pose_from_tag(tag_pose, node._H_TCP_stand2)

        px, py, pz, pqx, pqy, pqz, pqw = homogeneous_to_pos_quat(H_pick)
        lx, ly, lz, lqx, lqy, lqz, lqw = homogeneous_to_pos_quat(H_place)
        node.get_logger().info(f'Pick pose:  ({px:.4f}, {py:.4f}, {pz:.4f})')
        node.get_logger().info(f'Place pose: ({lx:.4f}, {ly:.4f}, {lz:.4f})')

        # 4. Pick from stand 1
        node.get_logger().info('========== Picking from stand 1 ==========')
        if not node.execute_pick(H_pick, tag_j1):
            node.get_logger().error('Pick failed. Aborting.')
            return

        # 5. Return to home
        node.get_logger().info('========== Returning home ==========')
        node.move_pose('home')
        time.sleep(0.5)

        # 6. Move to pre-place (5cm above stand 2 place pose)
        # node.get_logger().info('========== Moving to pre-place ==========')
        # node.move_joints([tag_j1, 0.0, 0.0, 0.0, 0.0, 0.0])
        # time.sleep(0.5)

        node.set_octomap_enabled(False)

        if not node.move_cartesian(lx, ly, lz + PRE_HEIGHT, lqx, lqy, lqz, lqw):
            node.get_logger().error('Failed to reach pre-place')
            return
        time.sleep(1.0)

        # 7. Admittance-controlled placement at stand 2
        node.get_logger().info('========== Admittance placement ==========')
        success, reason = node.run_admittance_place()

        if success:
            node.get_logger().info(f'Plate seated! {reason}')
        else:
            node.get_logger().warn(f'Placement incomplete: {reason}')

        # 8. Restore controllers
        node.cleanup_admittance()
        time.sleep(1.0)

        # 9. Open gripper to release plate
        node.get_logger().info('========== Releasing plate ==========')
        node.open_gripper()
        time.sleep(0.5)

        # 10. Retract to pre-place
        node.get_logger().info('========== Retracting to pre-place ==========')
        node.move_cartesian(lx, ly, lz + PRE_HEIGHT, lqx, lqy, lqz, lqw)
        time.sleep(0.5)

        # 11. Home
        node.get_logger().info('========== Returning home ==========')
        node.move_pose('home')

        node.get_logger().info('========== Pick and place complete ==========')

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
        node.cleanup_admittance()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
