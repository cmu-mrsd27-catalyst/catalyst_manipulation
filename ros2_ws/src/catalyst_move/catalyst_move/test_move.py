#!/usr/bin/env python3
"""Interactive test script for xArm6 arm movement and gripper control."""

import math
import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from catalyst_interfaces.srv import GripperCommand


PLANNING_GROUP = 'xarm6'
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'
EE_LINK = 'link_eef'

# Named poses (joint values in radians)
NAMED_POSES = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'hold_up': [0.0, 0.0, 0.0, 0.0, -math.pi / 2, 0.0],
}


def euler_to_quaternion(roll, pitch, yaw):
    """Convert roll/pitch/yaw (radians) to quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class TestMoveNode(Node):
    def __init__(self):
        super().__init__('test_move')
        self._cb_group = ReentrantCallbackGroup()
        self._move_action = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self._cb_group
        )
        self._gripper_client = self.create_client(
            GripperCommand,
            '/gripper_node/gripper_command',
            callback_group=self._cb_group,
        )

    # ------------------------------------------------------------------ arm
    def send_joint_goal(self, joint_values):
        """Send a joint-space goal to MoveGroup and wait for result."""
        if not self._move_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return False

        goal = MoveGroup.Goal()
        req = goal.request

        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0

        constraints = Constraints()
        for name, val in zip(JOINT_NAMES, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(f'Sending joint goal: {[f"{math.degrees(v):.1f}" for v in joint_values]} deg')
        future = self._move_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Joint goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error('No result received (timeout)')
            return False

        error_code = result.result.error_code.val
        if error_code == 1:  # SUCCESS
            self.get_logger().info('Motion succeeded!')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code: {error_code}')
            return False

    def send_pose_goal(self, x, y, z, roll, pitch, yaw):
        """Send a cartesian pose goal to MoveGroup and wait for result."""
        if not self._move_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return False

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        goal = MoveGroup.Goal()
        req = goal.request

        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0

        constraints = Constraints()

        # Position constraint
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = EE_LINK
        pc.weight = 1.0
        # Tolerance region: small sphere around target
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [0.001]
        pc.constraint_region.primitives.append(prim)
        target_pose = PoseStamped()
        target_pose.header.frame_id = BASE_FRAME
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw
        pc.constraint_region.primitive_poses.append(target_pose.pose)
        constraints.position_constraints.append(pc)

        # Orientation constraint
        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = EE_LINK
        oc.orientation.x = qx
        oc.orientation.y = qy
        oc.orientation.z = qz
        oc.orientation.w = qw
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        req.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(
            f'Sending pose goal: pos=({x:.3f}, {y:.3f}, {z:.3f}) '
            f'rpy=({math.degrees(roll):.1f}, {math.degrees(pitch):.1f}, {math.degrees(yaw):.1f}) deg'
        )
        future = self._move_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Pose goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error('No result received (timeout)')
            return False

        error_code = result.result.error_code.val
        if error_code == 1:  # SUCCESS
            self.get_logger().info('Motion succeeded!')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code: {error_code}')
            return False

    # -------------------------------------------------------------- gripper
    def call_gripper(self, action):
        """Call the gripper command service."""
        if not self._gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gripper service not available')
            return False

        req = GripperCommand.Request()
        req.command = json.dumps({'action': action})
        future = self._gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            self.get_logger().info(f'Gripper response: {future.result().response}')
            return True
        else:
            self.get_logger().error('Gripper service call failed')
            return False


def print_menu():
    print('\n=== Catalyst Move Test ===')
    print('1. Move to joint angles')
    print('2. Move to cartesian pose')
    print('3. Open gripper')
    print('4. Close gripper')
    print('5. Release gripper')
    print('6. Go to home pose')
    print('7. Go to hold_up pose')
    print('0. Exit')
    print()


def get_joint_angles():
    """Prompt user for 6 joint angles in degrees."""
    raw = input('Enter 6 joint angles in degrees (comma-separated): ')
    parts = [s.strip() for s in raw.split(',')]
    if len(parts) != 6:
        print(f'Expected 6 values, got {len(parts)}')
        return None
    try:
        return [math.radians(float(v)) for v in parts]
    except ValueError:
        print('Invalid number format')
        return None


def get_cartesian_pose():
    """Prompt user for x,y,z (meters) and roll,pitch,yaw (degrees)."""
    raw = input('Enter x,y,z (meters) and roll,pitch,yaw (degrees) comma-separated: ')
    parts = [s.strip() for s in raw.split(',')]
    if len(parts) != 6:
        print(f'Expected 6 values, got {len(parts)}')
        return None
    try:
        vals = [float(v) for v in parts]
    except ValueError:
        print('Invalid number format')
        return None
    x, y, z = vals[0], vals[1], vals[2]
    roll, pitch, yaw = math.radians(vals[3]), math.radians(vals[4]), math.radians(vals[5])
    return x, y, z, roll, pitch, yaw


def main():
    rclpy.init()
    node = TestMoveNode()

    try:
        while True:
            print_menu()
            choice = input('Choose an option: ').strip()

            if choice == '1':
                angles = get_joint_angles()
                if angles is not None:
                    node.send_joint_goal(angles)

            elif choice == '2':
                pose = get_cartesian_pose()
                if pose is not None:
                    node.send_pose_goal(*pose)

            elif choice == '3':
                node.call_gripper('open')

            elif choice == '4':
                node.call_gripper('close')

            elif choice == '5':
                node.call_gripper('release')

            elif choice == '6':
                print('Moving to home pose...')
                node.send_joint_goal(NAMED_POSES['home'])

            elif choice == '7':
                print('Moving to hold_up pose...')
                node.send_joint_goal(NAMED_POSES['hold_up'])

            elif choice == '0':
                print('Exiting.')
                break

            else:
                print('Invalid option.')

    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
