from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
   target_name = DeclareLaunchArgument(
      'target_name', default_value=TextSubstitution(text='lauv-simulator-1')
   )


   return LaunchDescription([
      target_name,
      Node(
         package='imcpy_ros_bridge',
         executable='imc2ros',
         namespace='from_lauv_simulator_1',
         name='imc2ros',
         output='screen',
         parameters=[{
            'target_name': LaunchConfiguration('target_name')
         }],
         arguments=['--ros-args', '--log-level', 'debug']
      ),
      Node(
         package='imcpy_ros_bridge',
         executable='ros2imc',
         namespace='to_lauv_simulator_1',
         name='ros2imc',
         output='screen',
         parameters=[{
            'target_name': LaunchConfiguration('target_name')
         }],
         arguments=['--ros-args', '--log-level', 'debug']
      ),
   ])
