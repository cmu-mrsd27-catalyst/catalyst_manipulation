#!/usr/bin/env python3
"""Test script for admittance-controlled well-plate placing.

Flow:
  1. Move arm to "home" named pose, open gripper
  2. User places well plate, close gripper
  3. Explore for AprilTag, compute place pose from H_TCP_TO
  4. Move to pre-place pose (5cm above place pose)
  5. Bias F/T sensor, run admittance placement downward
  6. Open gripper, retract to pre-place, go home

Usage (after demo.launch.py is running):
  ros2 run catalyst_execute test_place_admittance --ros-args \
    -p robot_ip:=192.168.1.212 -p ft_sensor_ip:=192.168.2.1
"""

import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
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


def quat_to_rotation_matrix(qx, qy, qz, qw):
    """Convert quaternion to 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


def rotation_matrix_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion (qx, qy, qz, qw)."""
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
    """Convert position/orientation dicts to 4x4 homogeneous matrix."""
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
    """Extract (x, y, z, qx, qy, qz, qw) from a 4x4 homogeneous matrix."""
    pos = H[:3, 3]
    qx, qy, qz, qw = rotation_matrix_to_quat(H[:3, :3])
    return pos[0], pos[1], pos[2], qx, qy, qz, qw


class PlaceAdmittanceTest(Node):
    def __init__(self):
        super().__init__('test_place_admittance')

        # Load H_TCP_TO from gripper_pose.json
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
        self._H_TCP_TAG = np.array(config['H_TCP_TO'])
        self.get_logger().info(f'Loaded H_TCP_TO from {config_path}')

        # World model subscription for tag detection
        self._world_model = None
        self._world_model_ts = None
        self.create_subscription(String, '/world_model', self._world_model_cb, 10)

        # Parameters
        self.declare_parameter('robot_ip', '192.168.1.212')
        self.declare_parameter('ft_sensor_ip', '192.168.2.1')
        self.declare_parameter('rate', 100.0)

        # Admittance gains
        self.declare_parameter('linear_gain', 0.005)      # m/s per N
        self.declare_parameter('force_deadzone', 1.0)      # N

        # Reference velocity: constant downward push in tool X
        # (gripper rotated -90° pitch, so flange X = downward at place pose)
        self.declare_parameter('ref_velocity_x', 0.005)    # m/s

        # Stopping criteria
        self.declare_parameter('place_force_threshold', 7.0)  # N — stop when X force exceeds this
        self.declare_parameter('max_place_distance', 0.1)     # m — max downward travel

        # Compliant axes: lateral (Y,Z) compliant, X follows reference velocity
        # [x, y, z, rx, ry, rz] in tool frame
        self.declare_parameter('compliant_axes', [0, 1, 1, 0, 0, 0])

        # Safety
        self.declare_parameter('max_linear_vel', 0.05)   # m/s

        # Service clients
        self._joint_client = self.create_client(JsonCommand, JOINT_SERVICE)
        self._cartesian_client = self.create_client(JsonCommand, CARTESIAN_SERVICE)
        self._gripper_client = self.create_client(JsonCommand, GRIPPER_SERVICE)
        self._arm = None
        self._ft_reader = None

    def _world_model_cb(self, msg: String):
        data = json.loads(msg.data)
        self._world_model_ts = data.get('timestamp')
        self._world_model = data

    def get_world_model(self):
        return (self._world_model_ts, self._world_model)

    def wait_for_world_model(self, timeout=10.0):
        self.get_logger().info('Waiting for world model data...')
        start = time.time()
        while self._world_model is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error('Timed out waiting for /world_model')
                return False
        self.get_logger().info('World model data received.')
        return True

    def explore_for_tag(self):
        explorer = TagExplorer(
            node=self,
            joint_client=self._joint_client,
            world_model_getter=self.get_world_model,
        )
        return explorer.search()

    def compute_place_pose(self, tag_pose):
        """Compute the place TCP pose from a detected tag pose using H_TCP_TO."""
        H_tag = pose_to_homogeneous(tag_pose['position'], tag_pose['orientation'])
        H_place = H_tag @ self._H_TCP_TAG
        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_place)
        self.get_logger().info(
            f'Place pose: pos=({x:.4f}, {y:.4f}, {z:.4f}) '
            f'quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})'
        )
        return H_place

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

    def move_cartesian(self, x, y, z, qx, qy, qz, qw, speed=0.1):
        return self._call(self._cartesian_client, {
            'x': x, 'y': y, 'z': z,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'speed': speed,
        })

    def open_gripper(self):
        return self._call(self._gripper_client, {'action': 'open'})

    def close_gripper(self):
        return self._call(self._gripper_client, {'action': 'close'})

    def wait_for_services(self):
        self.get_logger().info('Waiting for motion planner services...')
        self._joint_client.wait_for_service()
        self._cartesian_client.wait_for_service()
        self._gripper_client.wait_for_service()
        self.get_logger().info('All services ready.')

    # ── Admittance placement loop ──

    def run_admittance_place(self):
        """Run the admittance-controlled downward push until plate is seated."""
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

        # 1. Connect to F/T sensor
        self.get_logger().info(f'Connecting to F/T sensor at {ft_ip}...')
        self._ft_reader = OnRobotFTReader(ft_ip)
        self._ft_reader.start()

        # Bias sensor with arm stationary at pre-place pose
        self.get_logger().info('Biasing F/T sensor...')
        self._ft_reader.bias()
        self.get_logger().info('Bias complete.')

        # 2. Connect to xArm SDK and enter cartesian velocity mode (mode 5).
        #    Mode 5 != mode 1 (servo), so the HW plugin's write() returns early
        #    ("arm not ready") and won't send competing hold-position commands.
        #    Velocity mode also has no displacement limit — we command velocity,
        #    not position increments.
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
            f'Start pose: [{start_pose[0]:.1f}, {start_pose[1]:.1f}, {start_pose[2]:.1f}] mm'
        )

        # 4. Admittance loop with reference velocity
        self.get_logger().info(
            f'Admittance placing active! ref_vx={ref_vx*1000:.1f} mm/s, '
            f'stop at {place_force:.1f} N or {max_dist*1000:.0f} mm travel'
        )

        cycle_count = 0
        place_success = False
        reason = 'Interrupted'

        while rclpy.ok():
            loop_start = time.monotonic()

            # Read F/T
            ft = self._ft_reader.get_ft()  # [Fx,Fy,Fz,Tx,Ty,Tz] in N/Nm

            # Check stopping condition: X force (push axis) exceeds threshold
            # (gripper rotated -90° pitch, so flange X = downward at place pose)
            if abs(ft[0]) > place_force:
                reason = f'Force threshold reached: Fx={ft[0]:+.2f} N > {place_force} N'
                place_success = True
                break

            # Apply dead zone to forces
            for i in range(3):
                if abs(ft[i]) < force_dz:
                    ft[i] = 0.0

            # Compute compliance velocity from contact forces
            vel = np.zeros(6)
            vel[0:3] = linear_gain * ft[0:3]  # m/s from contact forces

            # Apply compliant axes mask (Y,Z lateral, X is push axis)
            vel *= compliant

            # Add reference downward velocity in tool X
            vel[0] += ref_vx

            # Clamp total linear velocity
            lin_speed = np.linalg.norm(vel[0:3])
            if lin_speed > max_lin_vel:
                vel[0:3] *= max_lin_vel / lin_speed

            # Check max distance via actual arm position
            code, actual_pose = self._arm.get_position()
            if code == 0:
                actual_travel = np.linalg.norm(
                    np.array(actual_pose[0:3]) - np.array(start_pose[0:3])
                ) / 1000.0  # mm → m
                if actual_travel > max_dist:
                    reason = f'Max distance reached: {actual_travel*1000:.1f} mm > {max_dist*1000:.0f} mm'
                    break

            # Send velocity command (mm/s and °/s in tool frame)
            vel_cmd = [
                vel[0] * 1000.0,  # m/s → mm/s
                vel[1] * 1000.0,
                vel[2] * 1000.0,
                0.0, 0.0, 0.0,   # no rotation
            ]
            ret = self._arm.vc_set_cartesian_velocity(
                vel_cmd, is_tool_coord=True, duration=-1
            )
            if ret != 0:
                self.get_logger().warn(f'vc_set ret={ret}')
                if self._arm.error_code != 0:
                    reason = f'xArm error: {self._arm.error_code}'
                    break

            # Log every 2 seconds
            cycle_count += 1
            if cycle_count % int(rate * 2) == 0:
                travel_mm = actual_travel * 1000.0 if code == 0 else -1
                self.get_logger().info(
                    f'F/T: [{ft[0]:+6.2f}, {ft[1]:+6.2f}, {ft[2]:+6.2f}] N  '
                    f'Vel: [{vel_cmd[0]:+.1f}, {vel_cmd[1]:+.1f}, {vel_cmd[2]:+.1f}] mm/s  '
                    f'Travel: {travel_mm:.1f} mm'
                )

            # Sleep remainder
            elapsed = time.monotonic() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Stop arm motion
        self._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_tool_coord=True)

        self.get_logger().info(f'Admittance stopped — reason: {reason}')
        return place_success, reason

    def cleanup_admittance(self):
        """Restore servo mode and let HW plugin auto-reactivate controllers.

        Mirrors guide_mode.py's disable_teach_mode() pattern:
          clean_error → clean_warn → motion_enable → set_mode(SERVO) → set_state(START)
        The HW plugin already set reactivate_controller_later_=true when it saw
        mode 5.  Once it sees mode=SERVO + state=START + no errors, its write()
        loop auto-reactivates all controllers.

        SDK disconnect happens AFTER reactivation (like guide_mode's finally block).
        """
        if self._ft_reader:
            self._ft_reader.close()
            self._ft_reader = None

        if self._arm:
            try:
                # Stop any velocity command
                self._arm.vc_set_cartesian_velocity([0, 0, 0, 0, 0, 0], is_tool_coord=True)
                time.sleep(0.3)

                # Match guide_mode.disable_teach_mode() exactly:
                # clear any errors, then set servo mode + start.
                self._arm.clean_error()
                self._arm.clean_warn()
                self._arm.motion_enable(True)
                self._arm.set_mode(XARM_MODE_SERVO)
                self._arm.set_state(XARM_STATE_START)
                self.get_logger().info(
                    f'Servo mode restored: mode={self._arm.mode}, state={self._arm.state}'
                )

                # Wait for HW plugin to see mode 1 + state 0 and auto-reactivate
                self.get_logger().info('Waiting for HW plugin to auto-reactivate controllers...')
                time.sleep(3.0)

                # Disconnect AFTER reactivation (same as guide_mode finally block)
                self._arm.disconnect()
                self.get_logger().info('SDK disconnected.')
            except Exception as e:
                self.get_logger().warn(f'Cleanup error: {e}')
            self._arm = None


def main():
    rclpy.init()
    node = PlaceAdmittanceTest()

    try:
        node.wait_for_services()

        if not node.wait_for_world_model():
            node.get_logger().error('No world model — aborting')
            return

        # Step 1: Go home
        node.get_logger().info('=== Moving to home ===')
        if not node.move_pose('home'):
            node.get_logger().error('Failed to move home')
            return

        # Step 2: Explore for AprilTag
        node.get_logger().info('=== Exploring for AprilTag ===')
        tag_pose, tag_j1 = node.explore_for_tag()
        if tag_pose is None:
            node.get_logger().error('Tag not found during exploration. Aborting.')
            return

        # Step 3: Compute place pose from tag + H_TCP_TO
        H_place = node.compute_place_pose(tag_pose)
        x, y, z, qx, qy, qz, qw = homogeneous_to_pos_quat(H_place)

        # Step 4: Go back to home, open gripper, ask user for well plate
        node.get_logger().info('=== Moving back to home ===')
        node.move_pose('home')
        time.sleep(0.5)

        node.get_logger().info('=== Opening gripper ===')
        node.open_gripper()
        time.sleep(0.5)

        node.get_logger().info('=== Place the well plate between the fingers ===')
        input('Press Enter when the well plate is in position...')
        node.get_logger().info('=== Closing gripper ===')
        node.close_gripper()
        time.sleep(1.0)

        # Step 5: Move to home facing the machine
        node.get_logger().info(f'=== Moving to home facing machine (j1={tag_j1:.1f} deg) ===')
        node._call(node._joint_client, {
            'joints': [tag_j1, 0.0, 0.0, 0.0, 0.0, 0.0],
            'speed': 0.1,
        })
        time.sleep(0.5)

        # Step 5: Move to pre-place pose (5cm above place pose)
        node.get_logger().info('=== Moving to pre-place pose (5cm above place) ===')
        if not node.move_cartesian(x, y, z + 0.05, qx, qy, qz, qw, speed=0.1):
            node.get_logger().error('Failed to reach pre-place pose')
            return

        time.sleep(1.0)  # settle before biasing

        # Step 6: Admittance-controlled placement
        node.get_logger().info('=== Starting admittance placement ===')
        success, reason = node.run_admittance_place()

        if success:
            node.get_logger().info(f'=== Plate seated! Reason: {reason} ===')
        else:
            node.get_logger().warn(f'=== Placement failed. Reason: {reason} ===')

        # Step 7: Restore trajectory controller
        node.cleanup_admittance()
        time.sleep(1.0)

        # Step 8: Open gripper to release plate
        node.open_gripper()
        time.sleep(0.5)

        # Step 9: Retract to pre-place pose
        node.get_logger().info('=== Retracting to pre-place pose ===')
        node.move_cartesian(x, y, z + 0.05, qx, qy, qz, qw, speed=0.1)

        # Step 10: Go home
        node.get_logger().info('=== Moving home ===')
        node.move_pose('home')

        node.get_logger().info('=== Place test complete ===')

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
        node.cleanup_admittance()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
