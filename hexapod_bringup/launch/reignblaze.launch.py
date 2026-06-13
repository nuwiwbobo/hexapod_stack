import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    description_pkg = get_package_share_directory('hexapod_description')

    xacro_file = os.path.join(description_pkg, 'urdf', 'reignblaze_sim.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    config_file = os.path.join(description_pkg, 'params', 'reignblaze.yaml')

    gait_node = Node(
        package='hexapod_gait',
        executable='gait_node_exec',
        name='gait_node',
        output='screen',
        parameters=[ParameterFile(config_file, allow_substs=True)],
    )

    ik_node = Node(
        package='hexapod_ik',
        executable='ik_node_exec',
        name='ik_node',
        output='screen',
        parameters=[ParameterFile(config_file, allow_substs=True)],
    )

    servo_node = Node(
        package='hexapod_servo',
        executable='servo_node_exec',
        name='servo_node',
        output='screen',
        parameters=[ParameterFile(config_file, allow_substs=True)],
    )

    control_node = Node(
        package='hexapod_control',
        executable='control_node_exec',
        name='control_node',
        output='screen',
        parameters=[ParameterFile(config_file, allow_substs=True)],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    activate_lifecycle_nodes = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', '''
                    for node in gait_node ik_node servo_node control_node; do
                        ros2 lifecycle set /$node configure
                        sleep 0.5
                        ros2 lifecycle set /$node activate
                        sleep 0.5
                    done
                    echo "All lifecycle nodes activated!"
                '''],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        gait_node,
        ik_node,
        servo_node,
        control_node,
        robot_state_publisher,
        activate_lifecycle_nodes,
    ])
