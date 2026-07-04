# =============================================================================
# Launch: Full 6-leg hexapod with teleop control
# =============================================================================
# Starts servo_node (18 servos), control_node (gait+IK), teleop keyboard.
#
# Usage:
#   ros2 launch hexapod_bringup reignblaze.launch.py
#
# Control:
#   Use keyboard to send cmd_vel commands:
#     i = forward, k = backward, j = left, l = right
#     u/o = rotate left/right, q = quit

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


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
            executable='control_node_exec',
            name='control_node',
            parameters=[{'config_file': config_file}],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
            output='screen'
        ),
    ])
