#!/usr/bin/env python3
"""Test script that moves the arm through a sequence of poses via arm_control services."""

import json
import math
import rclpy
from rclpy.node import Node
from catalyst_interfaces.srv import JsonCommand
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


JOINT_SERVICE = '/joint_command'
CARTESIAN_SERVICE = '/cartesian_command'
GRIPPER_SERVICE = '/gripper_command'


class TestExecuteNode(Node):
    def __init__(self):
        super().__init__('test_execute')
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.joint_state = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Example: pose of source_link w.r.t. target_link
        self.target_frame = "link_base"
        self.source_frame = "link_tcp"

        # self._joint_client = self.create_client(JsonCommand, JOINT_SERVICE)
        # self._cartesian_client = self.create_client(JsonCommand, CARTESIAN_SERVICE)
        self._gripper_client = self.create_client(JsonCommand, GRIPPER_SERVICE)

    def get_link_pose(self, target_frame: str, source_frame: str):

        tf = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time()
        )

        t = tf.transform.translation
        q = tf.transform.rotation

        return {
            "position": (t.x, t.y, t.z),
            "quaternion": (q.x, q.y, q.z, q.w)
        }

    def joint_states_callback(self, msg):
        self.joint_state = msg.position

    def return_joint(self):
        return [math.degrees(self.joint_state[0]),
                math.degrees(self.joint_state[1]),
                math.degrees(self.joint_state[2]),
                math.degrees(self.joint_state[3]),
                math.degrees(self.joint_state[4]),
                math.degrees(self.joint_state[5])]

    def wait_for_services(self):
        # self.get_logger().info('Waiting for arm_control services...')
        # self._joint_client.wait_for_service()
        # self._cartesian_client.wait_for_service()
        # self.get_logger().info('Waiting for gripper service...')
        self._gripper_client.wait_for_service()
        self.get_logger().info('All services ready.')

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

    def move_joints(self, joints, speed=1.0):
        return self._call(self._joint_client, {'joints': joints, 'speed': speed})

    def move_pose(self, pose_name, speed=1.0):
        return self._call(self._joint_client, {'pose': pose_name, 'speed': speed})

    def move_cartesian(self, x, y, z, qx, qy, qz, qw, speed=1.0):
        return self._call(self._cartesian_client, {
            'x': x, 'y': y, 'z': z,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'speed': speed
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
    # node.get_link_pose("link_base", "link_tcp")

    node.close_gripper()

    # 1. Go to home
    # node.move_pose('home', speed=0.2)
    # time.sleep(0.5)
    # node.open_gripper()
    # time.sleep(0.5)

    # print("Moving to 1st pose")
    # node.move_cartesian(0.54, -0.0813, 0.0908, 0.9991, -0.0408, 0.0038, 0.0124, 0.2)
    # time.sleep(1)

    # print("Moving to 2nd pose")
    # node.move_cartesian(0.342, -0.0, 0.157, 1.0, 0.0, 0.0, 0.0, 0.2)
    # time.sleep(1)

    # print("Moving to 3rd pose")
    # node.move_cartesian(0.4862, -0.1821, 0.1494, 0.8899, -0.4536, 0.0473, -0.0085, 0.2)
    # time.sleep(1)

    # print("Moving to 4th pose")
    # node.move_cartesian(0.4375, 0.2277, 0.0734, 0.9151, 0.4006, -0.0187, 0.0409, 0.2)
    # time.sleep(1)

    # print("Moving to 5th pose")
    # node.move_cartesian(0.0035, 0.4861, 0.0266, 0.7083, 0.7056, 0.018, 0.0103, 0.2)
    # time.sleep(1)

    # print("Moving to 6th pose")
    # node.move_cartesian(0.0887, -0.433, 0.0345, 0.7126, -0.7011, -0.0217, -0.0091, 0.2)
    # time.sleep(1)

    # node.move_pose('home', speed=0.2)

    # node.move_joints([-5.9, 31.23, -82.67, 2.59, 54.86, 0.44], speed = 0.2)
    # time.sleep(0.5)
    # node.move_joints([-5.6, 39.16, -77.06, 0.02, 38.86, 0.44], speed = 0.05)
    # time.sleep(0.5)
    # node.close_gripper()
    # time.sleep(0.5)
    # node.move_joints([-5.9, 31.23, -82.67, 2.59, 54.86, 0.44], speed = 0.05)
    # time.sleep(0.5)

    # 3. Move to specific joint angles (degrees)
    # node.move_joints([0, -30, 0, 0, -90, 0], 0.1)

    # 4. Move to a cartesian pose (meters + quaternion)
    #    Example: same as home TCP pose from tf2_echo
    # node.move_cartesian(0.348, 0.005, 0.064, 1.0, 0.007, 0.011, 0.0)

    # 5. Back to home
    # node.move_pose('home', speed=0.2)
    # node.open_gripper()

    node.get_logger().info('=== Test sequence complete ===')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
