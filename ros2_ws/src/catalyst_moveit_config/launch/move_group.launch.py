"""
MoveIt move_group launch file for Catalyst Manipulator.

Launches the move_group node which provides:
- Motion planning services
- Trajectory execution
- State monitoring
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load a YAML file from a package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    # Package directories
    moveit_config_pkg = get_package_share_directory('catalyst_moveit_config')
    description_pkg = get_package_share_directory('catalyst_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Robot description (URDF) - wrap in ParameterValue to handle as string
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            os.path.join(description_pkg, 'urdf', 'gripper_and_arm', 'arm_gripper_combined.urdf.xacro')
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # Robot description semantic (SRDF)
    robot_description_semantic_path = os.path.join(
        moveit_config_pkg, 'srdf', 'catalyst_manipulator.srdf'
    )
    with open(robot_description_semantic_path, 'r') as file:
        robot_description_semantic_content = file.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    # Kinematics configuration
    kinematics_yaml = load_yaml('catalyst_moveit_config', 'config/kinematics.yaml')

    # Joint limits configuration
    joint_limits_yaml = load_yaml('catalyst_moveit_config', 'config/joint_limits.yaml')
    joint_limits = {'robot_description_planning': joint_limits_yaml}

    # OMPL planning configuration (Jazzy format)
    ompl_planning_yaml = load_yaml('catalyst_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
        },
    }
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    # Trajectory execution configuration
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': False,
    }

    # Controllers configuration
    controllers_yaml = load_yaml('catalyst_moveit_config', 'config/controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # Planning scene monitor configuration
    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # 3D perception / OctoMap configuration
    sensors_3d_yaml = load_yaml('catalyst_moveit_config', 'config/sensors_3d.yaml')
    octomap_config = {
        'octomap_frame': 'link_base',
        'octomap_resolution': 0.005,
    }

    # move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor,
            sensors_3d_yaml,
            octomap_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        move_group_node,
    ])
