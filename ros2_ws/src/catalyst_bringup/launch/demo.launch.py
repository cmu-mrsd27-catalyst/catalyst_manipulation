"""
Full demo launch file for Catalyst Manipulator.

Launches everything needed for MoveIt planning and execution:
- Robot bringup (fake hardware + controllers)
- MoveIt move_group
- RViz with MoveIt plugin

Usage:
  ros2 launch catalyst_bringup demo.launch.py
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load a YAML file from a package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    # Package paths
    description_pkg = FindPackageShare('description')
    moveit_config_pkg = get_package_share_directory('moveit_config')
    bringup_pkg = FindPackageShare('catalyst_bringup')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(moveit_config_pkg, 'rviz', 'moveit.rviz'),
        description='Path to RViz config'
    )

    # Robot description (URDF) with fake hardware
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                description_pkg, 'urdf', 'gripper_and_arm', 'arm_gripper_combined.urdf.xacro'
            ]),
            ' ros2_control_plugin:=uf_robot_hardware/UFRobotFakeSystemHardware',
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # Robot description semantic (SRDF)
    srdf_path = os.path.join(moveit_config_pkg, 'srdf', 'catalyst_manipulator.srdf')
    with open(srdf_path, 'r') as file:
        robot_description_semantic = {'robot_description_semantic': file.read()}

    # Load MoveIt configs
    kinematics_yaml = load_yaml('moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('moveit_config', 'config/joint_limits.yaml')
    ompl_planning_yaml = load_yaml('moveit_config', 'config/ompl_planning.yaml')
    controllers_yaml = load_yaml('moveit_config', 'config/controllers.yaml')

    # MoveIt parameters
    joint_limits = {'robot_description_planning': joint_limits_yaml}
    ompl_planning = {'move_group': {'planning_plugin': 'ompl_interface/OMPLPlanner'}}
    ompl_planning['move_group'].update(ompl_planning_yaml)

    trajectory_execution = {
        'moveit_manage_controllers': False,  # Controllers managed by ros2_control
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

    # Controller config path
    controller_config = PathJoinSubstitution([
        bringup_pkg, 'config', 'ros2_controllers.yaml'
    ])

    # ========== Nodes ==========

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ros2_control controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    xarm6_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xarm6_traj_controller', '-c', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['bio_gripper_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # MoveIt move_group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
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

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Delay controller spawning
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

    # Delay move_group until controllers are ready
    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=xarm6_controller_spawner,
            on_exit=[move_group_node],
        )
    )

    # Delay RViz until move_group is ready
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=xarm6_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz_config,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delay_xarm6_controller,
        delay_gripper_controller,
        delay_move_group,
        delay_rviz,
    ])
