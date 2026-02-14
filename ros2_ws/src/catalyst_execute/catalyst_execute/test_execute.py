#!/usr/bin/env python3
"""Test script that moves the arm through a sequence of poses via arm_control services."""

import json

import rclpy
from rclpy.node import Node
from catalyst_interfaces.srv import GripperCommand


JOINT_SERVICE = '/arm_control/joint'
CARTESIAN_SERVICE = '/arm_control/cartesian'
GRIPPER_SERVICE = '/gripper_node/gripper_command'


class TestExecuteNode(Node):
    def __init__(self):
        super().__init__('test_execute')
        self._joint_client = self.create_client(GripperCommand, JOINT_SERVICE)
        self._cartesian_client = self.create_client(GripperCommand, CARTESIAN_SERVICE)
        # self._gripper_client = self.create_client(GripperCommand, GRIPPER_SERVICE)

    def wait_for_services(self):
        self.get_logger().info('Waiting for arm_control services...')
        self._joint_client.wait_for_service()
        self._cartesian_client.wait_for_service()
        self.get_logger().info('Waiting for gripper service...')
        # self._gripper_client.wait_for_service()
        self.get_logger().info('All services ready.')

    def _call(self, client, command):
        req = GripperCommand.Request()
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

    def move_joints(self, joints, speed=1.0):
        return self._call(self._joint_client, {'joints': joints, 'speed': speed})

    def move_pose(self, pose_name, speed=1.0):
        return self._call(self._joint_client, {'pose': pose_name, 'speed': speed})

    def move_cartesian(self, x, y, z, qx, qy, qz, qw, speed=1.0):
        return self._call(self._cartesian_client, {
            'x': x, 'y': y, 'z': z,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'speed': speed,
        })

    def open_gripper(self):
        return self._call(self._gripper_client, {'action': 'open'})

    def close_gripper(self):
        return self._call(self._gripper_client, {'action': 'close'})

    def release_gripper(self):
        return self._call(self._gripper_client, {'action': 'release'})


def main():
    rclpy.init()
    node = TestExecuteNode()
    node.wait_for_services()

    # --- Define your sequence here ---

    node.get_logger().info('=== Starting test sequence ===')

    # 1. Go to home
    node.move_pose('home', speed=0.2)
    # node.open_gripper()
    node.move_cartesian(0.5966, -0.0903, 0.14, 0.9974, -0.0569, -0.0439, 0.0105, speed=0.2)
    node.move_cartesian(0.5966, -0.0903, 0.0775, 0.9974, -0.0569, -0.0439, 0.0105, speed=0.2)
    # node.close_gripper

    # 3. Move to specific joint angles (degrees)
    # node.move_joints([0, -30, 0, 0, -90, 0])

    # 4. Move to a cartesian pose (meters + quaternion)
    #    Example: same as home TCP pose from tf2_echo
    # node.move_cartesian(0.348, 0.005, 0.064, 1.0, 0.007, 0.011, 0.0)

    # 5. Back to home
    node.move_pose('home', speed=0.2)

    node.get_logger().info('=== Test sequence complete ===')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
