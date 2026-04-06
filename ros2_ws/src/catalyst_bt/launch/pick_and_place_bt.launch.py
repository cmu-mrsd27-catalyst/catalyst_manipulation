from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bt_share = get_package_share_directory('catalyst_bt')
    tree_file = os.path.join(bt_share, 'trees', 'pick_and_place.xml')

    return LaunchDescription([
        Node(
            package='catalyst_bt',
            executable='bt_executor',
            name='bt_executor',
            output='screen',
            parameters=[{'tree_file': tree_file}],
        ),
    ])
