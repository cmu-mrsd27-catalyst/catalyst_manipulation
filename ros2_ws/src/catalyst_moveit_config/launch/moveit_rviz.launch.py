"""
MoveIt RViz launch file for Catalyst Manipulator.

Launches:
- move_group node (motion planning server)
- robot_state_publisher (TF broadcaster)
- RViz2 with MoveIt plugin
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
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(moveit_config_pkg, 'rviz', 'moveit.rviz'),
        description='Path to RViz config file'
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

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Joint state publisher GUI (for testing without real robot)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz_config,
        robot_state_publisher_node,
        joint_state_publisher_gui,
        move_group_node,
        rviz_node,
    ])
