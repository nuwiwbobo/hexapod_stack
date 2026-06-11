import os
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    description_pkg = get_package_share_directory('hexapod_description')

    xacro_file = os.path.join(description_pkg, 'urdf', 'reignblaze_model.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    config_file = os.path.join(description_pkg, 'params', 'reignblaze.yaml')

    container = ComposableNodeContainer(
        name='hexapod_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='hexapod_gait',
                plugin='hexapod_gait::GaitNode',
                name='gait_node',
                parameters=[
                    ParameterFile(config_file, allow_substs=True),
                ],
            ),
            ComposableNode(
                package='hexapod_ik',
                plugin='hexapod_ik::IkNode',
                name='ik_node',
                parameters=[
                    ParameterFile(config_file, allow_substs=True),
                ],
            ),
            ComposableNode(
                package='hexapod_servo',
                plugin='hexapod_servo::ServoNode',
                name='servo_node',
                parameters=[
                    ParameterFile(config_file, allow_substs=True),
                ],
            ),
            ComposableNode(
                package='hexapod_control',
                plugin='hexapod_control::ControlNode',
                name='control_node',
                parameters=[
                    ParameterFile(config_file, allow_substs=True),
                ],
            ),
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([
        container,
        robot_state_publisher,
    ])
