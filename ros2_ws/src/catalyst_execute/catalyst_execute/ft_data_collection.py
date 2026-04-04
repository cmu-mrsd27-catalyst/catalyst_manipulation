#!/usr/bin/env python3
"""F/T data collection during manual (guide mode) well-plate placement.

Collects force/torque sensor data while the user manually guides the arm
through various placement scenarios (normal, stuck on corners, etc.).

Flow:
  1. Open gripper → user places plate → close gripper
  2. Connect to F/T sensor and xArm SDK
  3. Enable guide mode
  4. Bias F/T sensor
  5. User presses Enter to start recording a scenario
  6. User manually places plate while F/T data is logged at 100Hz
  7. User presses Enter to stop recording
  8. Repeat for multiple scenarios
  9. Press 'q' to quit and disable guide mode

Saves CSV files to ~/ft_data_collection/ with scenario labels.

Usage:
  ros2 run catalyst_execute ft_data_collection --ros-args \
    -p robot_ip:=192.168.1.212 -p ft_sensor_ip:=192.168.2.1
"""

import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node

from catalyst_interfaces.srv import JsonCommand
from xarm.wrapper import XArmAPI

from catalyst_execute.sdk_admittance import OnRobotFTReader

GRIPPER_SERVICE = '/gripper_command'

XARM_MODE_TEACH = 2
XARM_STATE_START = 0
XARM_STATE_STOP = 4
XARM_MODE_SERVO = 1


class FTDataCollectionNode(Node):
    def __init__(self):
        super().__init__('ft_data_collection')

        self.declare_parameter('robot_ip', '192.168.1.212')
        self.declare_parameter('ft_sensor_ip', '192.168.2.1')
        self.declare_parameter('rate', 100.0)

        self._gripper_client = self.create_client(JsonCommand, GRIPPER_SERVICE)

        self._arm = None
        self._ft_reader = None

    def _call_gripper(self, action):
        req = JsonCommand.Request()
        req.command = json.dumps({'action': action})
        self.get_logger().info(f'Gripper: {action}')
        future = self._gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is not None:
            resp = json.loads(future.result().response)
            self.get_logger().info(f'Gripper result: {resp["message"]}')
            return resp['success']
        return False

    def open_gripper(self):
        return self._call_gripper('open')

    def close_gripper(self):
        return self._call_gripper('close')

    def connect_arm(self):
        robot_ip = self.get_parameter('robot_ip').value
        self.get_logger().info(f'Connecting to xArm at {robot_ip}...')
        self._arm = XArmAPI(robot_ip)
        time.sleep(0.5)
        if self._arm.error_code != 0:
            self._arm.clean_error()
        if self._arm.warn_code != 0:
            self._arm.clean_warn()
        self._arm.motion_enable(True)
        self.get_logger().info('xArm connected.')

    def connect_ft_sensor(self):
        ft_ip = self.get_parameter('ft_sensor_ip').value
        self.get_logger().info(f'Connecting to F/T sensor at {ft_ip}...')
        self._ft_reader = OnRobotFTReader(ft_ip)
        self._ft_reader.start()
        self.get_logger().info('F/T sensor connected and streaming.')

    def bias_ft_sensor(self):
        self.get_logger().info('Biasing F/T sensor...')
        self._ft_reader.bias()
        self.get_logger().info('F/T sensor biased.')

    def enable_guide_mode(self):
        if not self._arm:
            return False
        self.get_logger().info('Enabling guide mode...')
        self._arm.set_state(XARM_STATE_STOP)
        time.sleep(2.0)
        self._arm.set_mode(XARM_MODE_TEACH)
        self._arm.set_state(XARM_STATE_START)
        time.sleep(0.3)

        if self._arm.mode != XARM_MODE_TEACH:
            self.get_logger().warn(f'Teach mode did not take (mode={self._arm.mode}), retrying...')
            self._arm.clean_error()
            self._arm.clean_warn()
            self._arm.motion_enable(True)
            time.sleep(0.2)
            self._arm.set_mode(XARM_MODE_TEACH)
            self._arm.set_state(XARM_STATE_START)
            time.sleep(0.3)

        if self._arm.mode == XARM_MODE_TEACH:
            self.get_logger().info('Guide mode ENABLED.')
            return True
        else:
            self.get_logger().error(f'Guide mode FAILED (mode={self._arm.mode})')
            return False

    def disable_guide_mode(self):
        if not self._arm:
            return
        self.get_logger().info('Disabling guide mode...')
        self._arm.clean_error()
        self._arm.clean_warn()
        self._arm.motion_enable(True)
        self._arm.set_mode(XARM_MODE_SERVO)
        self._arm.set_state(XARM_STATE_START)
        time.sleep(1.0)
        self.get_logger().info('Servo mode restored.')

    def record_scenario(self, label, save_dir):
        """Record F/T data until user presses Enter."""
        rate = self.get_parameter('rate').value
        dt = 1.0 / rate

        filename = f'{label}.csv'
        filepath = os.path.join(save_dir, filename)

        with open(filepath, 'w') as f:
            f.write('time,fx,fy,fz,tx,ty,tz,pos_x,pos_y,pos_z\n')

            self.get_logger().info(f'Recording "{label}" → {filepath}')
            self.get_logger().info('Press Enter to stop recording...')

            t_start = time.monotonic()
            sample_count = 0

            import sys
            import select

            while True:
                loop_start = time.monotonic()

                ft = self._ft_reader.get_ft()

                # Get arm position (mm from SDK)
                pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
                code, pose = self._arm.get_position()
                if code == 0:
                    pos_x, pos_y, pos_z = pose[0], pose[1], pose[2]

                t_elapsed = time.monotonic() - t_start

                f.write(
                    f'{t_elapsed:.4f},'
                    f'{ft[0]:.4f},{ft[1]:.4f},{ft[2]:.4f},'
                    f'{ft[3]:.5f},{ft[4]:.5f},{ft[5]:.5f},'
                    f'{pos_x:.2f},{pos_y:.2f},{pos_z:.2f}\n'
                )

                sample_count += 1
                if sample_count % int(rate) == 0:
                    self.get_logger().info(
                        f'  [{t_elapsed:.1f}s] F:[{ft[0]:+6.2f},{ft[1]:+6.2f},{ft[2]:+6.2f}] N  '
                        f'T:[{ft[3]:+6.3f},{ft[4]:+6.3f},{ft[5]:+6.3f}] Nm'
                    )

                # Check for Enter key (non-blocking)
                if select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    break

                elapsed = time.monotonic() - loop_start
                if dt - elapsed > 0:
                    time.sleep(dt - elapsed)

        duration = time.monotonic() - t_start
        self.get_logger().info(
            f'Recorded {sample_count} samples ({duration:.1f}s) → {filepath}'
        )

    def cleanup(self):
        if self._ft_reader:
            self._ft_reader.close()
            self._ft_reader = None
        if self._arm:
            try:
                self._arm.disconnect()
            except Exception:
                pass
            self._arm = None


def main():
    rclpy.init()
    node = FTDataCollectionNode()

    save_dir = os.path.expanduser('~/ft_data_collection')
    os.makedirs(save_dir, exist_ok=True)

    try:
        # Wait for gripper service
        node.get_logger().info('Waiting for gripper service...')
        node._gripper_client.wait_for_service()

        # 1. Grasp the plate
        node.get_logger().info('=== Opening gripper ===')
        node.open_gripper()
        input('\nPlace the well plate in the gripper, then press Enter...\n')
        node.get_logger().info('=== Closing gripper ===')
        node.close_gripper()
        time.sleep(1.0)

        # 2. Connect arm and F/T sensor
        node.connect_arm()
        node.connect_ft_sensor()

        # 3. Enable guide mode
        if not node.enable_guide_mode():
            print('Failed to enable guide mode. Exiting.')
            node.cleanup()
            node.destroy_node()
            rclpy.shutdown()
            return

        # 4. Bias F/T sensor (arm should be stationary, holding plate)
        input('\nHold the arm still. Press Enter to bias F/T sensor...\n')
        node.bias_ft_sensor()

        # 5. Recording loop
        scenario_num = 0
        print('\n=== F/T Data Collection ===')
        print('You will manually guide the arm through placement scenarios.')
        print('For each scenario, enter a label and then press Enter to start/stop recording.')
        print('Type "q" to quit.\n')

        while True:
            label = input('Scenario label (or "q" to quit): ').strip()
            if label.lower() == 'q':
                break
            if not label:
                label = f'scenario_{scenario_num:03d}'

            input(f'Press Enter to START recording "{label}"...')

            # Re-bias before each scenario
            node.bias_ft_sensor()
            time.sleep(0.3)

            node.record_scenario(label, save_dir)
            scenario_num += 1

            print()

        # 6. Cleanup
        print('\nDisabling guide mode...')
        node.disable_guide_mode()

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
        node.disable_guide_mode()
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

    print(f'\nAll data saved to {save_dir}/')


if __name__ == '__main__':
    main()
