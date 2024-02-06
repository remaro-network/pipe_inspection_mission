from launch import LaunchDescription
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
                package='imcpy_trees',
                executable="followSingleRef",
                output='screen',
                emulate_tty=True,
            ),
        Node(
            namespace='lauv_simulator_1',
            package='imcpy_ros_bridge',
            executable="follow_single_reference_server",
            output='screen',
            emulate_tty=True,
        )
        ]
    )

