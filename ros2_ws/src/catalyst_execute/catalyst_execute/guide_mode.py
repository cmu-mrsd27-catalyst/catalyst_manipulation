#!/usr/bin/env python3
"""Guide mode: enable teach mode, manually move the arm, and read positions.

How it works with the UFRobotSystemHardware plugin
===================================================
The HW plugin's write() loop runs at 100 Hz and monitors the arm's mode/state.
If it detects mode != SERVO or state > 2, it auto-deactivates ALL controllers
(including joint_state_broadcaster) and sets reactivate_controller_later_ = true.
When the arm returns to SERVO mode + state <= 2, it auto-reactivates everything.

We MUST work with this mechanism, not against it:
  - To enter teach mode:  set teach mode via SDK → let the HW plugin detect it
    and auto-deactivate controllers.  Don't manually deactivate first — the HW
    plugin's write() would send servo commands before seeing the mode change.
  - To exit teach mode:   set SERVO mode + START state via SDK → the HW plugin
    detects the arm is ready and auto-reactivates all controllers.  Don't
    manually reactivate — that races with the HW plugin's own reactivation.
"""

import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from catalyst_interfaces.srv import JsonCommand
from xarm.wrapper import XArmAPI


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'
TCP_FRAME = 'link_tcp'

JOINT_SERVICE = '/joint_command'
CARTESIAN_SERVICE = '/cartesian_command'

XARM_MODE_SERVO = 1
XARM_MODE_TEACH = 2
XARM_STATE_START = 0
XARM_STATE_STOP = 4


class GuideModeNode(Node):
    def __init__(self):
        super().__init__('guide_mode')

        self.declare_parameter('robot_ip', '192.168.1.212')
        self._robot_ip = self.get_parameter('robot_ip').value

        # Service clients for catalyst_motion_planner
        self._joint_client = self.create_client(JsonCommand, JOINT_SERVICE)
        self._cartesian_client = self.create_client(JsonCommand, CARTESIAN_SERVICE)

        # TF2 for TCP pose
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Latest joint states
        self._joint_positions = {}
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10
        )

        # xArm SDK connection for mode switching
        self._arm = None

    def _joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

    def connect_arm(self):
        """Connect to xArm via Python SDK."""
        self.get_logger().info(f'Connecting to xArm at {self._robot_ip}...')
        try:
            self._arm = XArmAPI(self._robot_ip)
            self.get_logger().info('Connected to xArm.')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to xArm: {e}')
            return False

    def enable_teach_mode(self):
        """Switch the arm to teach (free-drive) mode.

        Sequence:
        1. Set arm state to STOP — the HW plugin's _xarm_is_ready_write()
           sees state > 2, returns false, _need_reset() returns true, write()
           skips servo commands and auto-deactivates all controllers.
        2. Wait for the HW plugin to finish deactivation and stop writing.
        3. Now safely set teach mode — no conflicting servo commands.
        """
        if not self._arm:
            self.get_logger().error('No xArm SDK connection')
            return False

        # Step 1: Put arm in STOP state so the HW plugin stops sending
        # servo commands and auto-deactivates controllers.
        self.get_logger().info('Setting arm to STOP state (HW plugin will auto-deactivate)...')
        self._arm.set_state(XARM_STATE_STOP)

        # Step 2: Wait for the HW plugin to detect state > 2, call
        # _need_reset() -> _deactivate_controller(), and for the controller
        # manager to finish the switch.
        time.sleep(2.0)

        # Step 3: Now safe to set teach mode — HW plugin's write() loop
        # returns early at _need_reset() and won't send servo commands.
        # NOTE: Do NOT call clean_error/clean_warn/motion_enable here — they
        # reset the arm mode to 0 (POSE), and the HW plugin might briefly see
        # a "ready" state before we set TEACH, causing it to auto-reactivate.
        # The arm was running normally before STOP, so no errors to clear.
        self._arm.set_mode(XARM_MODE_TEACH)
        self._arm.set_state(XARM_STATE_START)

        # Verify the mode actually took effect
        time.sleep(0.3)
        mode = self._arm.mode
        state = self._arm.state
        self.get_logger().info(f'Arm mode={mode}, state={state}')
        if mode != XARM_MODE_TEACH:
            self.get_logger().warn(
                f'set_mode(TEACH) did not take effect (mode={mode}). '
                f'Retrying with clean_error...'
            )
            self._arm.clean_error()
            self._arm.clean_warn()
            self._arm.motion_enable(True)
            time.sleep(0.2)
            self._arm.set_mode(XARM_MODE_TEACH)
            self._arm.set_state(XARM_STATE_START)
            time.sleep(0.3)
            self.get_logger().info(f'After retry: arm mode={self._arm.mode}')

        self.get_logger().info('Teach mode ENABLED — you can move the arm manually.')
        return True

    def disable_teach_mode(self):
        """Switch the arm back to servo mode so the HW plugin can take over.

        Set mode to SERVO (not POSE) and state to START — this is exactly what
        the HW plugin's on_activate() does.  The plugin's _xarm_is_ready_write()
        will return True and _activate_controller() will reactivate everything.
        """
        if not self._arm:
            self.get_logger().error('No xArm SDK connection')
            return False

        self._arm.clean_error()
        self._arm.clean_warn()
        self._arm.motion_enable(True)
        self._arm.set_mode(XARM_MODE_SERVO)
        self._arm.set_state(XARM_STATE_START)
        self.get_logger().info('Servo mode restored via SDK.')

        # Wait for the HW plugin to detect the arm is ready and auto-reactivate
        # all controllers (joint_state_broadcaster + xarm6_traj_controller).
        self.get_logger().info('Waiting for HW plugin to reactivate controllers...')
        time.sleep(3.0)
        self.get_logger().info('Controllers should be reactivated by HW plugin.')
        return True

    def disconnect_arm(self):
        """Disconnect the SDK connection."""
        if self._arm:
            self._arm.disconnect()
            self._arm = None

    def get_joint_positions(self):
        """Get joint positions — try /joint_states first, fall back to SDK."""
        # joint_state_broadcaster is auto-deactivated in teach mode, so
        # /joint_states may not be updating.  Use SDK as fallback.
        positions = {}
        for name in JOINT_NAMES:
            if name in self._joint_positions:
                positions[name] = self._joint_positions[name]

        if len(positions) == len(JOINT_NAMES):
            return positions

        # Fallback: read directly from xArm SDK
        if self._arm:
            code, angles = self._arm.get_servo_angle()
            if code == 0 and len(angles) >= 6:
                return {
                    name: math.radians(angles[i])
                    for i, name in enumerate(JOINT_NAMES)
                }
            self.get_logger().warn(f'SDK get_servo_angle failed (code={code})')

        self.get_logger().warn('Joint positions not available from any source')
        return None

    def get_tcp_pose(self):
        """Get TCP pose — try TF first, fall back to SDK."""
        try:
            t = self._tf_buffer.lookup_transform(BASE_FRAME, TCP_FRAME, rclpy.time.Time())
            tr = t.transform.translation
            rot = t.transform.rotation
            return {
                'x': tr.x, 'y': tr.y, 'z': tr.z,
                'qx': rot.x, 'qy': rot.y, 'qz': rot.z, 'qw': rot.w,
            }
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')

        # Fallback: read from SDK (returns [x, y, z, roll, pitch, yaw] in mm/deg)
        if self._arm:
            code, pose = self._arm.get_position()
            if code == 0 and len(pose) >= 6:
                from scipy.spatial.transform import Rotation
                r = Rotation.from_euler('xyz', pose[3:6], degrees=True)
                q = r.as_quat()  # [x, y, z, w]
                return {
                    'x': pose[0] / 1000.0,
                    'y': pose[1] / 1000.0,
                    'z': pose[2] / 1000.0,
                    'qx': q[0], 'qy': q[1], 'qz': q[2], 'qw': q[3],
                }
            self.get_logger().warn(f'SDK get_position failed (code={code})')

        self.get_logger().warn('TCP pose not available from any source')
        return None

    def _call_service(self, client, command):
        """Call a catalyst_motion_planner service."""
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

    def execute_joint_position(self, joints_deg, speed=0.2):
        """Send recorded joint position to /joint_command."""
        return self._call_service(self._joint_client, {'joints': joints_deg, 'speed': speed})

    def execute_cartesian_pose(self, pose, speed=0.2):
        """Send recorded TCP pose to /cartesian_command."""
        return self._call_service(self._cartesian_client, {
            'x': pose['x'], 'y': pose['y'], 'z': pose['z'],
            'qx': pose['qx'], 'qy': pose['qy'], 'qz': pose['qz'], 'qw': pose['qw'],
            'speed': speed,
        })


def print_menu():
    print('\n=== Guide Mode (Teach) ===')
    print('1. Get joint positions')
    print('2. Get TCP cartesian pose')
    print('3. Execute saved joint position (via /joint_command)')
    print('4. Execute saved cartesian pose (via /cartesian_command)')
    print('0. Exit (disables teach mode)')
    print()


def main():
    rclpy.init()
    node = GuideModeNode()

    # Connect to xArm and enable teach mode
    if not node.connect_arm():
        print('Cannot connect to xArm. Exiting.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if not node.enable_teach_mode():
        print('Failed to enable teach mode. Exiting.')
        node.disconnect_arm()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # Spin briefly to populate joint states and TF
    print('Waiting for joint states and TF...')
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    saved_joints = None
    saved_pose = None

    try:
        while True:
            print_menu()
            choice = input('Choose an option: ').strip()

            # Spin to get latest data
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)

            if choice == '1':
                joints = node.get_joint_positions()
                if joints:
                    deg_list = [round(math.degrees(joints[j]), 2) for j in JOINT_NAMES]
                    print('\nJoint positions:')
                    print(f'  Radians: {[f"{joints[j]:.4f}" for j in JOINT_NAMES]}')
                    print(f'  Degrees: {deg_list}')
                    cmd = {'joints': deg_list, 'speed': 0.2}
                    print(f'\n  /joint_command JSON: {json.dumps(cmd)}')
                    saved_joints = deg_list
                    print('  (Saved for execution with option 3)')
                else:
                    print('Joint positions not available.')

            elif choice == '2':
                pose = node.get_tcp_pose()
                if pose:
                    print(f'\nTCP pose ({BASE_FRAME} -> {TCP_FRAME}):')
                    print(f'  Position:    x={pose["x"]:.4f}  y={pose["y"]:.4f}  z={pose["z"]:.4f}  (meters)')
                    print(f'  Quaternion:  qx={pose["qx"]:.4f}  qy={pose["qy"]:.4f}  qz={pose["qz"]:.4f}  qw={pose["qw"]:.4f}')
                    sj = {k: round(v, 4) for k, v in pose.items()}
                    sj['speed'] = 0.2
                    print(f'\n  /cartesian_command JSON: {json.dumps(sj)}')
                    saved_pose = pose
                    print('  (Saved for execution with option 4)')
                else:
                    print('TCP pose not available.')

            elif choice == '3':
                if saved_joints is None:
                    print('No joint position saved. Use option 1 first.')
                else:
                    print('\nDisabling teach mode to execute...')
                    if not node.disable_teach_mode():
                        print('Failed to disable teach mode. Skipping execution.')
                        continue
                    print('Waiting for /joint_command service...')
                    node._joint_client.wait_for_service(timeout_sec=10.0)
                    node.execute_joint_position(saved_joints)
                    time.sleep(0.5)
                    print('Re-enabling teach mode...')
                    node.enable_teach_mode()

            elif choice == '4':
                if saved_pose is None:
                    print('No cartesian pose saved. Use option 2 first.')
                else:
                    print('\nDisabling teach mode to execute...')
                    if not node.disable_teach_mode():
                        print('Failed to disable teach mode. Skipping execution.')
                        continue
                    print('Waiting for /cartesian_command service...')
                    node._cartesian_client.wait_for_service(timeout_sec=10.0)
                    node.execute_cartesian_pose(saved_pose)
                    time.sleep(0.5)
                    print('Re-enabling teach mode...')
                    node.enable_teach_mode()

            elif choice == '0':
                print('Disabling teach mode...')
                node.disable_teach_mode()
                break

            else:
                print('Invalid option.')

    except KeyboardInterrupt:
        print('\nDisabling teach mode...')
        node.disable_teach_mode()
    finally:
        node.disconnect_arm()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
