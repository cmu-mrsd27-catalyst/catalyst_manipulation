"""
Dynamixel AX-18A Gripper Node for Catalyst Manipulator.

Provides:
- ~/gripper_command service (catalyst_interfaces/srv/GripperCommand)
  for JSON-based open/close commands
- Continuous joint state publishing on /joint_states with real servo position

left_finger_joint is handled automatically via the URDF mimic relationship.
"""

import json
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from catalyst_interfaces.srv import JsonCommand
from catalyst_gripper.gripper_controller import AX18AGripper, GripperConfig


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # Declare parameters
        self.declare_parameter('config_path', '')
        self.declare_parameter('max_position', 0.037)
        self.declare_parameter('publish_rate', 30.0)

        config_path = self.get_parameter('config_path').value
        self._max_pos = self.get_parameter('max_position').value
        publish_rate = self.get_parameter('publish_rate').value

        # Initialize gripper controller
        if not config_path:
            config_path = os.path.join(
                get_package_share_directory('catalyst_gripper'),
                'config', 'gripper_params.yaml',
            )
        self._cfg = GripperConfig.from_yaml(config_path)
        self.get_logger().info(f'Loaded gripper config from {config_path}')
        self._gripper = AX18AGripper(self._cfg)

        try:
            self._gripper.connect()
            self.get_logger().info(
                f'Connected to Dynamixel servo on {self._cfg.devicename} '
                f'(ID={self._cfg.dxl_id})'
            )
        except RuntimeError as e:
            self.get_logger().error(f'Failed to connect to gripper: {e}')
            raise

        # Lock for servo access (service runs in a separate thread)
        self._servo_lock = threading.Lock()

        # Callback groups: allow service to run concurrently with timer
        self._service_cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = ReentrantCallbackGroup()

        # Service: /gripper_command
        self._srv = self.create_service(
            JsonCommand,
            '/gripper_command',
            self._handle_gripper_command,
            callback_group=self._service_cb_group,
        )

        # Publisher: joint states for robot_state_publisher
        self._joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )

        # Publisher: gripper state for world model
        self._gripper_state_pub = self.create_publisher(
            String, '/gripper_state', 10
        )

        # Timer: publish joint states at configured rate
        self._timer = self.create_timer(
            1.0 / publish_rate,
            self._publish_joint_state,
            callback_group=self._timer_cb_group,
        )

        self.get_logger().info(
            f'Gripper node started (publish_rate={publish_rate} Hz)'
        )

    def _ticks_to_meters(self, ticks):
        """Convert servo ticks to meters for the joint state."""
        ratio = (ticks - self._cfg.pos_min) / (self._cfg.pos_max - self._cfg.pos_min)
        return max(0.0, min(self._max_pos, ratio * self._max_pos))

    def _handle_gripper_command(self, request, response):
        """Handle GripperCommand service calls."""
        try:
            cmd = json.loads(request.command)
        except json.JSONDecodeError as e:
            response.response = json.dumps({
                'success': False,
                'message': f'Invalid JSON: {e}',
            })
            return response

        action = cmd.get('action', '').lower()

        if action == 'open':
            target_pos = cmd.get('position', None)
            self.get_logger().info(
                f'Opening gripper'
                + (f' to position {target_pos}' if target_pos is not None else '')
            )
            try:
                with self._servo_lock:
                    self._gripper.open_gripper(target_pos)
                    data = self._gripper.get_data()
                response.response = json.dumps({
                    'success': True,
                    'message': 'gripper opened',
                    'position': data['position'],
                })
            except RuntimeError as e:
                response.response = json.dumps({
                    'success': False,
                    'message': str(e),
                })

        elif action == 'close':
            self.get_logger().info('Closing gripper')
            try:
                with self._servo_lock:
                    contact = self._gripper.close_gripper()
                    data = self._gripper.get_data()
                msg = 'contact detected' if contact else 'no object detected'
                response.response = json.dumps({
                    'success': contact,
                    'message': msg,
                    'position': data['position'],
                })
            except RuntimeError as e:
                response.response = json.dumps({
                    'success': False,
                    'message': str(e),
                })

        elif action == 'release':
            self.get_logger().info('Releasing gripper')
            try:
                with self._servo_lock:
                    self._gripper.release()
                    data = self._gripper.get_data()
                response.response = json.dumps({
                    'success': True,
                    'message': 'gripper released',
                    'position': data['position'],
                })
            except RuntimeError as e:
                response.response = json.dumps({
                    'success': False,
                    'message': str(e),
                })

        else:
            response.response = json.dumps({
                'success': False,
                'message': f"Unknown action '{action}'. Use 'open', 'close', or 'release'.",
            })

        return response

    def _publish_joint_state(self):
        """Publish current gripper joint state and gripper state."""
        try:
            with self._servo_lock:
                data = self._gripper.get_data()
                grasping = self._gripper.is_grasping()
            meters = self._ticks_to_meters(data['position'])
        except RuntimeError:
            return  # skip this cycle if read fails

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['right_finger_joint']
        msg.position = [meters]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self._joint_state_pub.publish(msg)

        # Determine gripper state
        if grasping:
            state = 'holding'
        elif meters < 0.005:
            state = 'closed'
        elif meters > 0.03:
            state = 'open'
        else:
            state = 'moving'

        gripper_msg = String()
        gripper_msg.data = json.dumps({
            'position_ticks': data['position'],
            'position_meters': round(meters, 6),
            'velocity': data.get('velocity', 0),
            'torque': data.get('torque', 0),
            'is_grasping': grasping,
            'state': state,
        })
        self._gripper_state_pub.publish(gripper_msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down gripper node')
        try:
            self._gripper.release()
        except Exception:
            pass
        self._gripper.close_port()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
