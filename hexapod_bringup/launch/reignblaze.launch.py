# =============================================================================
# Task 8: Launch file for Reignblaze Hexapod
# =============================================================================
# Starts the control orchestrator node with parameters from reignblaze.yaml
#
# Usage:
#   ros2 launch hexapod_bringup reignblaze.launch.py
#
# With keyboard teleop:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
#
# Prerequisites:
#   1. Complete Tasks 1-7 (servo, msgs, ik, gait, control)
#   2. Uncomment all CMakeLists.txt targets
#   3. Build: colcon build --packages-select hexapod_stack

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'reignblaze.yaml'
    )

    return LaunchDescription([
        Node(
            package='hexapod_control',
            executable='control_node_exec',
            name='hexapod_control',
            parameters=[config_file],
            output='screen'
        ),
    ])
