#!/usr/bin/env python3
"""Guide mode: enable teach mode, manually move the arm, and read positions."""

import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from controller_manager_msgs.srv import SwitchController
from catalyst_interfaces.srv import JsonCommand
from xarm.wrapper import XArmAPI


JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'
TCP_FRAME = 'link_tcp'
ARM_CONTROLLER = 'xarm6_traj_controller'
JSB_CONTROLLER = 'joint_state_broadcaster'

JOINT_SERVICE = '/joint_command'
CARTESIAN_SERVICE = '/cartesian_command'

XARM_MODE_TEACH = 2
XARM_MODE_POSITION = 0
XARM_STATE_READY = 0


class GuideModeNode(Node):
    def __init__(self):
        super().__init__('guide_mode')

        self.declare_parameter('robot_ip', '192.168.1.212')
        self._robot_ip = self.get_parameter('robot_ip').value

        # Controller manager client
        self._switch_ctrl = self.create_client(
            SwitchController, '/controller_manager/switch_controller'
        )

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

    def _switch_controller(self, activate=None, deactivate=None, retries=5):
        """Activate/deactivate controllers via controller_manager, with retries."""
        if not self._switch_ctrl.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('controller_manager switch_controller service not available')
            return False

        for attempt in range(retries):
            req = SwitchController.Request()
            req.activate_controllers = activate or []
            req.deactivate_controllers = deactivate or []
            req.strictness = SwitchController.Request.STRICT
            future = self._switch_ctrl.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None and future.result().ok:
                return True
            self.get_logger().warn(
                f'Controller switch attempt {attempt + 1}/{retries} failed, retrying...'
            )
            time.sleep(1.0)

        self.get_logger().error('Failed to switch controllers after all retries')
        return False

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
        """Deactivate arm controller, ensure joint_state_broadcaster is active, then teach mode."""
        self.get_logger().info(f'Deactivating {ARM_CONTROLLER}...')
        self._switch_controller(deactivate=[ARM_CONTROLLER])

        # Ensure joint_state_broadcaster stays active so we can read positions
        self.get_logger().info(f'Ensuring {JSB_CONTROLLER} is active...')
        self._switch_controller(activate=[JSB_CONTROLLER])

        if not self._arm:
            return False
        self._arm.set_mode(XARM_MODE_TEACH)
        self._arm.set_state(XARM_STATE_READY)
        self.get_logger().info('Teach mode ENABLED — you can move the arm manually.')
        return True

    def disable_teach_mode(self):
        """Switch xArm back to position mode, then reactivate arm controller."""
        if not self._arm:
            return False
        self._arm.set_mode(XARM_MODE_POSITION)
        self._arm.set_state(XARM_STATE_READY)
        self.get_logger().info('Teach mode DISABLED.')

        self.get_logger().info(f'Reactivating {ARM_CONTROLLER}...')
        self._switch_controller(activate=[ARM_CONTROLLER])
        return True

    def disconnect_arm(self):
        """Disconnect the SDK connection."""
        if self._arm:
            self._arm.disconnect()
            self._arm = None

    def get_joint_positions(self):
        missing = [j for j in JOINT_NAMES if j not in self._joint_positions]
        if missing:
            self.get_logger().warn(f'Missing joint states for: {missing}')
            return None
        return {name: self._joint_positions[name] for name in JOINT_NAMES}

    def get_tcp_pose(self):
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

    node.enable_teach_mode()

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
                    print(f'\nDisabling teach mode to execute...')
                    node.disable_teach_mode()
                    time.sleep(1.0)
                    # Wait for services
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
                    print(f'\nDisabling teach mode to execute...')
                    node.disable_teach_mode()
                    time.sleep(1.0)
                    # Wait for services
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
