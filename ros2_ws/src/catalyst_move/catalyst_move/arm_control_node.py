#!/usr/bin/env python3
"""ROS2 service node for xArm6 cartesian and joint position control."""

import math
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
)
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from catalyst_interfaces.srv import GripperCommand


PLANNING_GROUP = 'xarm6'
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
BASE_FRAME = 'link_base'
EE_LINK = 'link_tcp'

NAMED_POSES = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'hold_up': [0.0, 0.0, 0.0, 0.0, -math.pi / 2, 0.0],
}



def _resp(success, message):
    return json.dumps({'success': success, 'message': message})


class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control')
        self._cb = ReentrantCallbackGroup()

        self._move_action = ActionClient(
            self, MoveGroup, 'move_action', callback_group=self._cb
        )

        self.create_service(
            GripperCommand, '~/cartesian', self._handle_cartesian,
            callback_group=self._cb,
        )
        self.create_service(
            GripperCommand, '~/joint', self._handle_joint,
            callback_group=self._cb,
        )

        self.get_logger().info('Arm control services ready:')
        self.get_logger().info('  ~/cartesian  — {x, y, z, qx, qy, qz, qw}')
        self.get_logger().info('  ~/joint      — {joints: [...]} or {pose: "name"}')

    # --------------------------------------------------------- helpers

    def _wait_for_future(self, future, timeout=60.0):
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)
        return future.done()

    def _execute_goal(self, goal):
        if not self._move_action.wait_for_server(timeout_sec=5.0):
            return False, 'MoveGroup action server not available'

        future = self._move_action.send_goal_async(goal)
        if not self._wait_for_future(future, timeout=10.0):
            return False, 'Timed out waiting for goal acceptance'

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False, 'Goal rejected by MoveGroup'

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout=60.0):
            return False, 'Timed out waiting for motion result'

        result = result_future.result()
        if result is None:
            return False, 'No result received'

        error_code = result.result.error_code.val
        if error_code == 1:
            return True, 'Motion succeeded'
        return False, f'Motion failed (error code {error_code})'

    # ------------------------------------------------------ cartesian

    def _handle_cartesian(self, request, response):
        """Handle cartesian pose service call.

        JSON: {"x": 0.3, "y": 0.0, "z": 0.5, "qx": 0.0, "qy": 0.707, "qz": 0.0, "qw": 0.707}
              (x/y/z in meters, qx/qy/qz/qw as quaternion)
        """
        try:
            cmd = json.loads(request.command)
        except json.JSONDecodeError as e:
            response.response = _resp(False, f'Invalid JSON: {e}')
            return response

        required = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        missing = [k for k in required if k not in cmd]
        if missing:
            response.response = _resp(False, f'Missing fields: {missing}')
            return response

        try:
            x = float(cmd['x'])
            y = float(cmd['y'])
            z = float(cmd['z'])
            qx = float(cmd['qx'])
            qy = float(cmd['qy'])
            qz = float(cmd['qz'])
            qw = float(cmd['qw'])
        except (ValueError, TypeError) as e:
            response.response = _resp(False, f'Invalid values: {e}')
            return response

        self.get_logger().info(
            f'Cartesian goal: pos=({x:.3f}, {y:.3f}, {z:.3f}) '
            f'quat=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})'
        )

        speed = max(0.01, min(1.0, float(cmd.get('speed', 1.0))))

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = speed
        req.max_acceleration_scaling_factor = speed

        constraints = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = EE_LINK
        pc.weight = 1.0
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [0.001]
        pc.constraint_region.primitives.append(prim)
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        pc.constraint_region.primitive_poses.append(pose.pose)
        constraints.position_constraints.append(pc)

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

        success, message = self._execute_goal(goal)
        response.response = _resp(success, message)
        return response

    # ---------------------------------------------------------- joint

    def _handle_joint(self, request, response):
        """Handle joint position service call.

        JSON (angles in degrees):
            {"joints": [0, 0, 0, 0, -90, 0]}
        Or named pose:
            {"pose": "home"}
        """
        try:
            cmd = json.loads(request.command)
        except json.JSONDecodeError as e:
            response.response = _resp(False, f'Invalid JSON: {e}')
            return response

        if 'pose' in cmd:
            name = cmd['pose']
            if name not in NAMED_POSES:
                response.response = _resp(
                    False, f'Unknown pose "{name}". Available: {list(NAMED_POSES.keys())}'
                )
                return response
            joint_values = list(NAMED_POSES[name])
            self.get_logger().info(f'Joint goal: named pose "{name}"')
        elif 'joints' in cmd:
            try:
                angles_deg = [float(v) for v in cmd['joints']]
            except (ValueError, TypeError) as e:
                response.response = _resp(False, f'Invalid joint values: {e}')
                return response
            if len(angles_deg) != 6:
                response.response = _resp(
                    False, f'Expected 6 joint values, got {len(angles_deg)}'
                )
                return response
            joint_values = [math.radians(v) for v in angles_deg]
            self.get_logger().info(f'Joint goal: {[f"{v:.1f}" for v in angles_deg]} deg')
        else:
            response.response = _resp(
                False, 'Provide "joints" (6 angles in degrees) or "pose" (name)'
            )
            return response

        speed = max(0.01, min(1.0, float(cmd.get('speed', 1.0))))

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = PLANNING_GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = speed
        req.max_acceleration_scaling_factor = speed

        constraints = Constraints()
        for jname, val in zip(JOINT_NAMES, joint_values):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        success, message = self._execute_goal(goal)
        response.response = _resp(success, message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
