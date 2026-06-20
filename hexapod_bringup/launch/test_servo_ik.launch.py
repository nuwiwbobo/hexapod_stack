# =============================================================================
# Launch: Servo + IK test (3 servos, Left Rear leg)
# =============================================================================
# Starts servo_node and ik_node with 3-servo config.
#
# Usage:
#   ros2 launch hexapod_bringup test_servo_ik.launch.py
#
# Then send foot positions:
#   ros2 topic pub /foot_target geometry_msgs/msg/PointStamped "{
#     header: {frame_id: 'body'},
#     point: {x: 0.05, y: 0.12, z: -0.10}
#   }"

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('hexapod_bringup'),
        'config', 'test_3servo.yaml'
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
            package='hexapod_ik',
            executable='ik_node_exec',
            name='ik_node',
            parameters=[{'config_file': config_file}],
            output='screen'
        ),
    ])
