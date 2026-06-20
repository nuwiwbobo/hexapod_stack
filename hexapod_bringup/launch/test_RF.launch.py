import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('hexapod_bringup'), 'config', 'test_RF.yaml')
    return LaunchDescription([
        Node(package='hexapod_servo', executable='servo_node_exec', name='servo_node', parameters=[{'config_file': config}], output='screen'),
        Node(package='hexapod_ik', executable='ik_node_exec', name='ik_node', parameters=[{'config_file': config}], output='screen'),
    ])
