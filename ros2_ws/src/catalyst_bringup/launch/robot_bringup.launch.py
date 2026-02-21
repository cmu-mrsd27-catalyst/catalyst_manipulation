"""
Robot bringup launch file for Catalyst Manipulator.

Launches:
- robot_state_publisher (URDF -> TF)
- ros2_control controller_manager (with fake hardware)
- joint_state_broadcaster
- xarm6_traj_controller
- bio_gripper_controller
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    description_pkg = FindPackageShare('catalyst_description')
    bringup_pkg = FindPackageShare('catalyst_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    declare_ros2_control_plugin = DeclareLaunchArgument(
        'ros2_control_plugin',
        default_value='uf_robot_hardware/UFRobotFakeSystemHardware',
        description='ros2_control hardware plugin'
    )

    # Robot description (URDF)
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                description_pkg, 'urdf', 'gripper_and_arm', 'arm_gripper_combined.urdf.xacro'
            ]),
            ' ros2_control_plugin:=', ros2_control_plugin,
        ]),
        value_type=str
    )

    # Controller config
    controller_config = PathJoinSubstitution([
        bringup_pkg, 'config', 'ros2_controllers.yaml'
    ])

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # ros2_control controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controller_config,
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # Spawn joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Spawn xarm6_traj_controller (after joint_state_broadcaster)
    xarm6_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xarm6_traj_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # Spawn bio_gripper_controller (after joint_state_broadcaster)
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bio_gripper_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # Delay controller spawning until joint_state_broadcaster is ready
    delay_xarm6_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[xarm6_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_ros2_control_plugin,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_xarm6_controller,
        delay_gripper_controller,
    ])
