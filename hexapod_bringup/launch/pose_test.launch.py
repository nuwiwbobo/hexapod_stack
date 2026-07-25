import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('hexapod_bringup'),
        'config', 'reignblaze.yaml'
    )

    return LaunchDescription([
        Node(
            package='hexapod_servo',
            executable='servo_node_exec',
            name='servo_node',
            parameters=[{'config_file': config_file}],
            output='screen'
        ),
        Node(
            package='hexapod_control',
            executable='pose_test_node',
            name='pose_test_node',
            parameters=[{'config_file': config_file}],
            output='screen',
            prefix=['xterm', '-e'],
        ),
    ])
