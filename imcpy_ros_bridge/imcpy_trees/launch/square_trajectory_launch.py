from launch import LaunchDescription
import launch_ros
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    """
    Launcher for the tutorial.

    Returns:
        the launch description
    """
    return LaunchDescription([
        Node(
            namespace='lauv_simulator_1',
            package='imcpy_ros_bridge',
            executable="imc2ros",
            output='screen',
            emulate_tty=True,
            # arguments=['--ros-args', '--log-level', 'DEBUG']
            ),    
        launch_ros.actions.Node(
                namespace='lauv_simulator_1',
                package='imcpy_trees',
                executable="square",
                output='screen',
                emulate_tty=True,
                # arguments=['--ros-args', '--log-level', 'DEBUG']
            ),
        ]
    )

