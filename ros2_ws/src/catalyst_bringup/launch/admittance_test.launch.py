"""
Admittance control test launch file.

Run AFTER demo.launch.py is already up. This switches from the trajectory
controller to the ros2_control admittance controller, making the arm
compliant to external forces measured by the F/T sensor.

Usage:
  ros2 launch catalyst_bringup admittance_test.launch.py

To switch back to normal trajectory control:
  ros2 control switch_controllers --deactivate admittance_controller --activate xarm6_traj_controller
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo


def generate_launch_description():
    # Switch from trajectory controller to admittance controller.
    # The admittance_controller was loaded --inactive by demo.launch.py.
    switch_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--deactivate', 'xarm6_traj_controller',
            '--activate', 'admittance_controller',
        ],
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg='Switching from xarm6_traj_controller to admittance_controller...'),
        LogInfo(msg='The arm will become compliant to external forces.'),
        LogInfo(msg='To switch back: ros2 control switch_controllers --deactivate admittance_controller --activate xarm6_traj_controller'),
        TimerAction(period=1.0, actions=[switch_controllers]),
    ])
