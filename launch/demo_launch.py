from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    my_package_dir = get_package_share_directory('path_smoothing_ros')
    return LaunchDescription([
        Node(
            package='path_smoothing_ros',
            namespace='path_smoothing_ros',
            executable='demo',
            name='path_smoothing_demo'
        ),
        Node(package='rviz2',
             executable='rviz2',
        )
    ])
