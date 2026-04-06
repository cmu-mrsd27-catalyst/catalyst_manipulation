"""Utility to restart ros2_control_node and re-spawn controllers.

Used after SDK-based operations (admittance, velocity control) that bypass
ros2_control. Kills the existing ros2_control_node, starts a fresh one with
the current URDF, and spawns the required controllers.
"""

import os
import subprocess
import tempfile
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters


def restart_ros2_control(node, logger):
    """Kill ros2_control_node, restart it, and re-spawn controllers.

    Args:
        node: An rclpy Node (used to create service clients and spin).
        logger: An rclpy logger for status messages.

    Returns:
        True if successful, False otherwise.
    """
    logger.info('=== Restarting ros2_control_node ===')

    # 1. Get robot_description from robot_state_publisher
    logger.info('Fetching robot_description...')
    param_client = node.create_client(
        GetParameters, '/robot_state_publisher/get_parameters'
    )
    if not param_client.wait_for_service(timeout_sec=5.0):
        logger.error('Cannot reach robot_state_publisher')
        return False

    req = GetParameters.Request()
    req.names = ['robot_description']
    future = param_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if future.result() is None:
        logger.error('Failed to get robot_description')
        return False
    urdf = future.result().values[0].string_value

    # 2. Get controller config path
    bringup_pkg = get_package_share_directory('catalyst_bringup')
    controller_config = os.path.join(bringup_pkg, 'config', 'ros2_controllers_real.yaml')

    # 3. Write URDF to temp param file
    param_data = {
        'ros2_control_node': {
            'ros__parameters': {
                'robot_description': urdf
            }
        }
    }
    param_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='ros2ctrl_'
    )
    yaml.dump(param_data, param_file, default_flow_style=False)
    param_file.close()

    # 4. Kill ros2_control_node
    logger.info('Killing ros2_control_node...')
    subprocess.run(['pkill', '-9', '-f', 'ros2_control_node'], timeout=5)
    time.sleep(3)

    # 5. Start new ros2_control_node
    logger.info('Starting new ros2_control_node...')
    subprocess.Popen(
        [
            'ros2', 'run', 'controller_manager', 'ros2_control_node',
            '--ros-args',
            '--params-file', param_file.name,
            '--params-file', controller_config,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(3)

    # 6. Spawn controllers
    controllers = [
        'joint_state_broadcaster',
        'xarm6_traj_controller',
        'force_torque_sensor_broadcaster',
    ]
    for ctrl in controllers:
        logger.info(f'Spawning {ctrl}...')
        result = subprocess.run(
            ['ros2', 'run', 'controller_manager', 'spawner',
             ctrl, '-c', '/controller_manager'],
            timeout=30, capture_output=True, text=True,
        )
        if result.returncode != 0:
            logger.error(f'{ctrl} spawn failed: {result.stderr}')
            if ctrl != 'force_torque_sensor_broadcaster':
                return False

    logger.info('ros2_control_node restarted and controllers spawned.')
    time.sleep(2)

    # Clean up temp file
    try:
        os.unlink(param_file.name)
    except OSError:
        pass

    return True
