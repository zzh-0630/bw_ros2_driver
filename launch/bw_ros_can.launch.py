import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bw_ros2_driver')
    params_file = os.path.join(pkg_share, 'config', 'can_params_ros2.yaml')

    return LaunchDescription([
        Node(
            package='bw_ros2_driver',
            executable='bw_node_can',
            name='bw_node_can',
            output='screen',
            parameters=[params_file],
        ),
    ])
