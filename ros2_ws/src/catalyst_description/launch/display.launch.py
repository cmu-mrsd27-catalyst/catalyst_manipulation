"""
Display launch file for the Catalyst manipulator.

Loads the composed URDF (xArm-6 + bio gripper) and opens RViz
with joint_state_publisher_gui so you can verify the model.

Usage:
  ros2 launch catalyst_description display.launch.py
  ros2 launch catalyst_description display.launch.py use_gui:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare("catalyst_description")

    # ---- Declare arguments ----
    use_gui_arg = DeclareLaunchArgument(
        "use_gui", default_value="true",
        description="Launch joint_state_publisher_gui (set false for headless)",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([pkg_share, "rviz", "display.rviz"]),
        description="Path to RViz config file",
    )

    # ---- Build robot_description from xacro ----
    xacro_file = PathJoinSubstitution(
        [pkg_share, "urdf", "gripper_and_arm", "arm_gripper_combined.urdf.xacro"]
    )
    robot_description_content = Command(
        [FindExecutable(name="xacro"), " ", xacro_file]
    )

    # ---- Nodes ----
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(robot_description_content, value_type=str)}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription([
        use_gui_arg,
        rviz_config_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
