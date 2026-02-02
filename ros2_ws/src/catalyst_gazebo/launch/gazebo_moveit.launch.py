"""
Gazebo + MoveIt Launch File for Catalyst Manipulator
====================================================

Complete simulation setup with Gazebo physics and MoveIt motion planning.

This launch file:
1. Starts Gazebo with the catalyst_workspace world
2. Spawns the robot on the robot stand
3. Loads ros2_control controllers
4. Starts MoveIt move_group for motion planning
5. Launches RViz with MoveIt plugin

USAGE:
    ros2 launch catalyst_gazebo gazebo_moveit.launch.py

ARGUMENTS:
    world       : Path to world file (default: catalyst_workspace.world)
    paused      : Start Gazebo paused (default: false)
    gui         : Show Gazebo GUI (default: true)
    rviz        : Launch RViz (default: true)
    use_sim_time: Use simulation time (default: true)

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    gazebo_pkg = get_package_share_directory('catalyst_gazebo')
    description_pkg = get_package_share_directory('description')
    bringup_pkg = get_package_share_directory('catalyst_bringup')
    moveit_config_pkg = get_package_share_directory('moveit_config')
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

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
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
    rviz = LaunchConfiguration('rviz')
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

    # Load SRDF
    robot_description_semantic_content = Command([
        FindExecutable(name='cat'), ' ',
        PathJoinSubstitution([moveit_config_pkg, 'srdf', 'catalyst_manipulator.srdf'])
    ])

    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)
    }

    # Kinematics configuration
    kinematics_yaml = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')

    # OMPL planning configuration
    ompl_planning_yaml = os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml')

    # Joint limits configuration
    joint_limits_yaml = os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml')

    # Controllers configuration for MoveIt
    moveit_controllers_yaml = os.path.join(moveit_config_pkg, 'config', 'controllers.yaml')

    # RViz configuration
    rviz_config = os.path.join(moveit_config_pkg, 'rviz', 'moveit.rviz')

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

    # Controller configuration path for Gazebo ros2_control
    controller_config = os.path.join(bringup_pkg, 'config', 'ros2_controllers.yaml')

    # Start Gazebo server with controller parameters
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': world,
            'pause': paused,
            'extra_gazebo_args': '--ros-args --params-file ' + controller_config,
        }.items(),
    )

    # Start Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py')
        ]),
        condition=IfCondition(gui),
    )

    # Spawn robot in Gazebo (on top of stand at Z=0.8)
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
            '-z', '0.8',
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

    # Arm trajectory controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xarm6_traj_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Gripper controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bio_gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # MoveIt move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
            joint_limits_yaml,
            moveit_controllers_yaml,
            {'use_sim_time': use_sim_time},
            {'planning_scene_monitor_options': {
                'robot_description': 'robot_description',
                'joint_state_topic': '/joint_states',
            }},
            # Don't manage controllers - Gazebo ros2_control does that
            {'moveit_manage_controllers': False},
            {'trajectory_execution': {
                'allowed_execution_duration_scaling': 1.2,
                'allowed_goal_duration_margin': 0.5,
                'allowed_start_tolerance': 0.01,
            }},
        ],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'robot_description': robot_description_content},
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(rviz),
    )

    # ==================== EVENT HANDLERS ====================
    # Sequence: spawn_robot -> joint_state_broadcaster -> controllers -> move_group

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

    # Start MoveIt after controllers are loaded
    delayed_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    # Start RViz after move_group
    delayed_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription([
        # Environment
        gazebo_model_path,

        # Arguments
        world_arg,
        paused_arg,
        gui_arg,
        rviz_arg,
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

        # MoveIt (delayed)
        delayed_move_group,
        delayed_rviz,
    ])
