"""
Gazebo Simulation Launch File for Catalyst Manipulator
======================================================

Launches Gazebo Classic with the robot in a workspace environment.

This launch file:
1. Starts Gazebo with the catalyst_workspace world
2. Spawns the robot on top of the robot stand
3. Loads ros2_control controllers

USAGE:
    ros2 launch catalyst_gazebo gazebo.launch.py

ARGUMENTS:
    world       : Path to world file (default: catalyst_workspace.world)
    paused      : Start Gazebo paused (default: false)
    gui         : Show Gazebo GUI (default: true)
    use_sim_time: Use simulation time (default: true)

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    gazebo_pkg = get_package_share_directory('catalyst_gazebo')
    description_pkg = get_package_share_directory('description')
    bringup_pkg = get_package_share_directory('catalyst_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Set Gazebo model path to include our models
    models_path = os.path.join(gazebo_pkg, 'models')
    gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        [EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''), ':', models_path]
    )

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gazebo_pkg, 'worlds', 'catalyst_workspace.world'),
        description='Path to Gazebo world file'
    )

    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start Gazebo paused'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Show Gazebo GUI'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Configuration
    world = LaunchConfiguration('world')
    paused = LaunchConfiguration('paused')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot description with Gazebo plugins
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                description_pkg, 'urdf', 'gripper_and_arm', 'arm_gripper_combined.urdf.xacro'
            ]),
            ' use_gazebo:=true',
            ' ros2_control_plugin:=gazebo_ros2_control/GazeboSystem',
        ]),
        value_type=str
    )

    # ==================== NODES ====================

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time},
        ],
    )

    # Start Gazebo server directly (avoiding parameter inheritance issues)
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    # Start Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        ]),
        condition=IfCondition(gui),
    )

    # Spawn robot in Gazebo
    # Robot is placed on top of the stand (Z = 0.8m)
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_catalyst_manipulator',
        output='screen',
        arguments=[
            '-entity', 'catalyst_manipulator',
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
        gazebo_model_path,

        # Arguments
        world_arg,
        paused_arg,
        gui_arg,
        use_sim_time_arg,

        # Gazebo
        gazebo_server,
        gazebo_client,

        # Robot
        robot_state_publisher,
        spawn_robot,

        # Controllers (delayed)
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        delayed_gripper_controller,
    ])
