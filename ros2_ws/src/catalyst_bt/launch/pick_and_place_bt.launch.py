from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Starts bt_executor as a long-lived node. Trigger tasks via /bt_execute (JsonCommand)."""
    return LaunchDescription([
        Node(
            package='catalyst_bt',
            executable='bt_executor',
            name='bt_executor',
            output='screen',
        ),
    ])
