"""
Unified demo launch file for Catalyst Manipulator.

Supports three modes:
  sim:=false   (default) — Real xArm hardware via UFRobotSystemHardware
  sim:=fake              — Fake hardware for quick MoveIt testing
  sim:=gazebo            — Full Gazebo simulation

Usage:
  ros2 launch catalyst_bringup demo.launch.py sim:=fake
  ros2 launch catalyst_bringup demo.launch.py sim:=gazebo
  ros2 launch catalyst_bringup demo.launch.py robot_ip:=192.168.1.212
"""

import copy
import os
import re
import subprocess
import sys
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name, file_path):
    """Load a YAML file from a package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def _get_launch_arg(name, default):
    """Parse a launch argument from sys.argv (key:=value format)."""
    for arg in sys.argv:
        if arg.startswith(f'{name}:='):
            return arg.split(':=', 1)[1]
    return default


def generate_launch_description():
    # ── Resolve mode at description-generation time ──
    # This avoids OpaqueFunction, which can leak parameters to ExecuteProcess
    # children (causing gazebo_ros2_control parser errors).
    sim_mode = _get_launch_arg('sim', 'false')
    robot_ip = _get_launch_arg('robot_ip', '192.168.1.212')

    is_real = (sim_mode == 'false')
    is_fake = (sim_mode == 'fake')
    is_gazebo = (sim_mode == 'gazebo')
    use_sim_time = is_gazebo

    # ── Package paths ──
    description_pkg = get_package_share_directory('description')
    bringup_pkg = get_package_share_directory('catalyst_bringup')
    moveit_config_pkg = get_package_share_directory('moveit_config')

    # ── Determine ros2_control plugin ──
    if is_real:
        plugin = 'uf_robot_hardware/UFRobotSystemHardware'
    elif is_fake:
        plugin = 'uf_robot_hardware/UFRobotFakeSystemHardware'
    else:
        plugin = 'gazebo_ros2_control/GazeboSystem'

    # ── Generate URDF via xacro ──
    xacro_file = os.path.join(
        description_pkg, 'urdf', 'gripper_and_arm', 'arm_gripper_combined.urdf.xacro'
    )
    xacro_args = [
        'xacro', xacro_file,
        f'ros2_control_plugin:={plugin}',
        f'use_gazebo:={str(is_gazebo).lower()}',
        f'add_gripper_ros2_control:={str(not is_real).lower()}',
    ]
    if is_real:
        xacro_args.append(f'robot_ip:={robot_ip}')

    urdf_content = subprocess.check_output(xacro_args).decode('utf-8')

    # Strip XML comments — gazebo_ros2_control's parameter parser chokes on them
    urdf_content = re.sub(r'<!--.*?-->', '', urdf_content, flags=re.DOTALL)

    # Write temp URDF file for Gazebo spawning
    urdf_path = None
    if is_gazebo:
        urdf_file = tempfile.NamedTemporaryFile(
            mode='w', prefix='catalyst_robot_', suffix='.urdf', delete=False
        )
        urdf_file.write(urdf_content)
        urdf_file.close()
        urdf_path = urdf_file.name

    # ── Controller config ──
    if is_real:
        controller_config_path = os.path.join(bringup_pkg, 'config', 'ros2_controllers_real.yaml')
    else:
        controller_config_path = os.path.join(bringup_pkg, 'config', 'ros2_controllers.yaml')

    # ── MoveIt config ──
    srdf_path = os.path.join(moveit_config_pkg, 'srdf', 'catalyst_manipulator.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = {'robot_description_kinematics': load_yaml('moveit_config', 'config/kinematics.yaml')}
    joint_limits_yaml = load_yaml('moveit_config', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('moveit_config', 'config/ompl_planning.yaml')
    controllers_yaml = load_yaml('moveit_config', 'config/controllers.yaml')

    # In real mode, remove bio_gripper_controller from MoveIt controllers
    if is_real:
        controllers_yaml = copy.deepcopy(controllers_yaml)
        if 'controller_names' in controllers_yaml:
            controllers_yaml['controller_names'] = [
                c for c in controllers_yaml['controller_names']
                if c != 'bio_gripper_controller'
            ]
        controllers_yaml.pop('bio_gripper_controller', None)

    joint_limits = {'robot_description_planning': joint_limits_yaml}
    ompl_planning = {'move_group': {'planning_plugin': 'ompl_interface/OMPLPlanner'}}
    ompl_planning['move_group'].update(ompl_planning_yaml)

    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ── RViz config ──
    rviz_config = os.path.join(moveit_config_pkg, 'rviz', 'moveit.rviz')
    rviz = LaunchConfiguration('rviz')

    # ==================== NODES ====================
    actions = []

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': urdf_content},
            {'use_sim_time': use_sim_time},
        ],
    )
    actions.append(robot_state_publisher_node)

    # ros2_control_node (not needed for Gazebo — plugin handles it)
    if not is_gazebo:
        controller_manager_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': urdf_content},
                controller_config_path,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        )
        actions.append(controller_manager_node)

    # Joint state publisher for real mode
    if is_real:
        joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('joint_states', '/joint_states')],
            output='screen',
        )
        actions.append(joint_state_publisher_node)

        # Gripper node (real hardware only — sim modes use bio_gripper_controller)
        gripper_pkg = get_package_share_directory('catalyst_gripper')
        gripper_config_path = os.path.join(gripper_pkg, 'config', 'gripper_params.yaml')
        gripper_node = Node(
            package='catalyst_gripper',
            executable='gripper_node',
            name='gripper_node',
            output='screen',
            parameters=[{'config_path': gripper_config_path}],
        )
        actions.append(gripper_node)

    # ── Gazebo nodes ──
    if is_gazebo:
        gazebo_pkg = get_package_share_directory('catalyst_gazebo')
        models_path = os.path.join(gazebo_pkg, 'models')
        plugin_path = os.path.join(get_package_prefix('xarm_gazebo'), 'lib')

        gazebo_model_path = SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            [models_path, ':', EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')]
        )
        gazebo_plugin_path = SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            [EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value=''), ':', plugin_path]
        )
        actions.extend([gazebo_model_path, gazebo_plugin_path])

        world_file = os.path.join(gazebo_pkg, 'worlds', 'catalyst_workspace.world')

        gazebo_server = ExecuteProcess(
            cmd=['env', '-u', 'RCL_ARGUMENTS', '-u', 'ROS_ARGS',
                 'gzserver', '--verbose', world_file,
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            additional_env={'RCL_ARGUMENTS': '', 'ROS_ARGS': ''},
        )

        gazebo_client = ExecuteProcess(
            cmd=['env', '-u', 'RCL_ARGUMENTS', '-u', 'ROS_ARGS', 'gzclient'],
            output='screen',
            additional_env={'RCL_ARGUMENTS': '', 'ROS_ARGS': ''},
        )

        spawn_robot = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_catalyst_manipulator',
            output='screen',
            arguments=[
                '-entity', 'catalyst_manipulator',
                '-file', urdf_path,
                '-x', '0.0', '-y', '0.0', '-z', '0.8',
                '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        actions.extend([gazebo_server, gazebo_client, spawn_robot])

    # ── Controller spawners ──
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    xarm6_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xarm6_traj_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── MoveIt move_group ──
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': urdf_content},
            robot_description_semantic,
            kinematics_yaml,
            joint_limits,
            ompl_planning,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Scene Manager (collision objects for planning scene) ──
    gazebo_pkg_dir = get_package_share_directory('catalyst_gazebo')
    world_objects_yaml_path = os.path.join(gazebo_pkg_dir, 'config', 'world_objects.yaml')

    scene_manager_node = Node(
        package='catalyst_motion_planner',
        executable='scene_manager',
        name='scene_manager',
        output='screen',
        parameters=[
            {'robot_description': urdf_content},
            robot_description_semantic,
            {'use_sim_time': use_sim_time},
            {'world_objects_yaml': world_objects_yaml_path},
            {'base_frame_z': 0.80},
        ],
    )

    # ── Motion Planner (C++ MoveGroupInterface node) ──
    motion_planner_node = Node(
        package='catalyst_motion_planner',
        executable='motion_planner',
        name='motion_planner',
        output='screen',
        parameters=[
            {'robot_description': urdf_content},
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': use_sim_time},
            {'enable_gripper_service': not is_real},
        ],
    )

    # ── RViz ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'robot_description': urdf_content},
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(rviz),
    )

    # ==================== EVENT SEQUENCING ====================
    if is_gazebo:
        # Gazebo: spawn_robot → joint_state_broadcaster → controllers → move_group + rviz
        delayed_jsb = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
        actions.append(delayed_jsb)
    else:
        # Fake/Real: launch joint_state_broadcaster immediately
        actions.append(joint_state_broadcaster_spawner)

    # arm controller after joint_state_broadcaster
    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[xarm6_controller_spawner],
        )
    )
    actions.append(delayed_arm_controller)

    # gripper controller after joint_state_broadcaster (sim modes only)
    if not is_real:
        gripper_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['bio_gripper_controller', '--controller-manager', '/controller_manager'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )
        delayed_gripper_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[gripper_controller_spawner],
            )
        )
        actions.append(delayed_gripper_controller)

    # move_group + rviz after arm controller is ready
    delayed_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=xarm6_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    delayed_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=xarm6_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    # Motion planner needs move_group to be fully up (joint_states flowing).
    # Launch it a few seconds after move_group starts.
    delayed_motion_planner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=xarm6_controller_spawner,
            on_exit=[TimerAction(period=5.0, actions=[motion_planner_node])],
        )
    )

    # Scene manager also needs move_group for PlanningSceneInterface
    delayed_scene_manager = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=xarm6_controller_spawner,
            on_exit=[TimerAction(period=5.0, actions=[scene_manager_node])],
        )
    )

    actions.append(delayed_move_group)
    actions.append(delayed_rviz)
    actions.append(delayed_motion_planner)
    actions.append(delayed_scene_manager)

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            choices=['false', 'fake', 'gazebo'],
            description='Simulation mode: false (real hardware), fake, or gazebo',
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.212',
            description='xArm IP address (only used when sim:=false)',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz',
        ),
    ] + actions)
