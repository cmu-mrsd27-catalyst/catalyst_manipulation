"""
Dynamixel AX-18A Gripper Node for Catalyst Manipulator.

Publishes right_finger_joint position to /joint_states so that
robot_state_publisher computes gripper TF and MoveIt sees the
correct collision state. left_finger_joint is handled automatically
via the URDF mimic relationship.

Subscribes to gripper commands on ~/command (std_msgs/Float64)
to open/close the gripper.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # Declare parameters (loaded from gripper_params.yaml)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('servo_id', 1)
        self.declare_parameter('min_position', 0.0)
        self.declare_parameter('max_position', 0.037)
        self.declare_parameter('publish_rate', 30.0)

        self._port = self.get_parameter('port').value
        self._baud_rate = self.get_parameter('baud_rate').value
        self._servo_id = self.get_parameter('servo_id').value
        self._min_pos = self.get_parameter('min_position').value
        self._max_pos = self.get_parameter('max_position').value
        publish_rate = self.get_parameter('publish_rate').value

        # Current joint position (start closed)
        self._current_position = 0.0

        # TODO: Initialize Dynamixel SDK
        # from dynamixel_sdk import PortHandler, PacketHandler
        # self._port_handler = PortHandler(self._port)
        # self._packet_handler = PacketHandler(1.0)  # Protocol 1.0 for AX-18A
        # if not self._port_handler.openPort():
        #     self.get_logger().error(f'Failed to open port {self._port}')
        #     return
        # if not self._port_handler.setBaudRate(self._baud_rate):
        #     self.get_logger().error(f'Failed to set baud rate {self._baud_rate}')
        #     return
        # self.get_logger().info(f'Dynamixel port {self._port} opened at {self._baud_rate} baud')

        # Publisher: joint states for robot_state_publisher
        self._joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )

        # Subscriber: gripper commands (position in meters)
        self._command_sub = self.create_subscription(
            Float64, '~/command', self._command_callback, 10
        )

        # Timer: publish joint states at configured rate
        self._timer = self.create_timer(1.0 / publish_rate, self._publish_joint_state)

        self.get_logger().info('Gripper node started (Dynamixel SDK not yet initialized)')

    def _command_callback(self, msg: Float64):
        """Handle gripper position command."""
        target = max(self._min_pos, min(self._max_pos, msg.data))

        # TODO: Convert meters to Dynamixel position ticks and write to servo
        # goal_position = self._meters_to_ticks(target)
        # self._packet_handler.write2ByteTxRx(
        #     self._port_handler, self._servo_id, ADDR_GOAL_POSITION, goal_position
        # )

        self._current_position = target
        self.get_logger().debug(f'Gripper command: {target:.4f} m')

    def _publish_joint_state(self):
        """Publish current gripper joint state."""
        # TODO: Read actual position from Dynamixel servo
        # present_position, _, _ = self._packet_handler.read2ByteTxRx(
        #     self._port_handler, self._servo_id, ADDR_PRESENT_POSITION
        # )
        # self._current_position = self._ticks_to_meters(present_position)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['right_finger_joint']
        msg.position = [self._current_position]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self._joint_state_pub.publish(msg)

    def destroy_node(self):
        # TODO: Close Dynamixel port
        # if hasattr(self, '_port_handler'):
        #     self._port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
