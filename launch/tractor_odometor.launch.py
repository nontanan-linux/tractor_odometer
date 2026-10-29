import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tractor_odometer'),
        'config',
        'tractor_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='tractor_odometer',
            executable='odometer_node',
            name='tractor_odometer',
            output='screen',
            parameters=[config]
        )
    ])
