import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Find the path to your config file inside the install directory
    config = os.path.join(
        get_package_share_directory('catalyst_vision'),
        'config',
        'apriltag_config.yaml'
    )

    # RealSense camera launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        )
    )

    # Static transform from link_eef to camera_color_optical_frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_eye_tf',
        arguments=[
            '--x', '0.06474074351789574',
            '--y', '-0.02079755382336791',
            '--z', '-0.043061004427683805',
            '--qx', '0.49950804885434386',
            '--qy', '0.4944359129754877',
            '--qz', '0.4926799984086548',
            '--qw', '0.5131191443104329',
            '--frame-id', 'link_eef',
            '--child-frame-id', 'camera_color_optical_frame',
        ]
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters=[config],
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info')
        ]
    )

    return LaunchDescription([
        realsense_launch,
        static_tf,
        apriltag_node,
    ])