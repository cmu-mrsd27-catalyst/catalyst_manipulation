"""
Gz Harmonic Simulation Launch File for Catalyst Manipulator
============================================================

Launches Gz Harmonic with the robot in a workspace environment.

This launch file:
1. Starts Gz Harmonic with the catalyst_workspace world
2. Spawns the robot on top of the robot stand
3. Bridges /clock from Gz to ROS 2
4. Loads ros2_control controllers

USAGE:
    ros2 launch catalyst_gazebo gazebo.launch.py

ARGUMENTS:
    world       : Path to world file (default: catalyst_workspace.sdf)
    use_sim_time: Use simulation time (default: true)

"""

import os
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    gazebo_pkg = get_package_share_directory('catalyst_gazebo')
    description_pkg = get_package_share_directory('catalyst_description')

    # Set GZ_SIM_RESOURCE_PATH to include our models
    models_path = os.path.join(gazebo_pkg, 'models')
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [models_path, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]
    )

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gazebo_pkg, 'worlds', 'catalyst_workspace.sdf'),
        description='Path to Gz world file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Configuration
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot description with Gz Harmonic plugins
    xacro_file = os.path.join(
        description_pkg, 'urdf', 'gripper_and_arm', 'arm_gripper_combined.urdf.xacro'
    )
    urdf_content = subprocess.check_output([
        'xacro', xacro_file,
        'use_gazebo:=true',
        'ros2_control_plugin:=gz_ros2_control/GazeboSimSystem',
    ]).decode('utf-8')
    # Strip XML comments to avoid ROS arg parser errors
    urdf_content = re.sub(r'<!--.*?-->', '', urdf_content, flags=re.DOTALL)
    robot_description = {'robot_description': urdf_content}

    # ==================== NODES ====================

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Start Gz Harmonic using ros_gz_sim
    world_path = os.path.join(gazebo_pkg, 'worlds', 'catalyst_workspace.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': '-r -v 3 {} --physics-engine gz-physics-bullet-featherstone-plugin'.format(world_path),
        }.items(),
    )

    # Spawn robot in Gz Harmonic
    # Robot is placed on top of the stand (Z = 0.8m)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_catalyst_manipulator',
        output='screen',
        arguments=[
            '-name', 'catalyst_manipulator',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.8',  # Top of robot stand
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gz-ROS bridge for clock (required — Gz Harmonic doesn't auto-bridge clock)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Arm trajectory controller (delayed until joint_state_broadcaster is ready)
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xarm6_traj_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gripper controller (delayed until joint_state_broadcaster is ready)
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bio_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Delay controller spawning until after robot is spawned
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delayed_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        # Environment
        gz_resource_path,

        # Arguments
        world_arg,
        use_sim_time_arg,

        # Gz Harmonic
        gazebo,

        # Clock bridge
        gz_bridge,

        # Robot
        robot_state_publisher,
        spawn_robot,

        # Controllers (delayed)
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        delayed_gripper_controller,
    ])
