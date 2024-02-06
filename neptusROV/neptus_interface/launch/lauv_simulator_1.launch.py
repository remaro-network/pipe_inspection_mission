import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
   target_name = DeclareLaunchArgument(
      'target_name', default_value=TextSubstitution(text='lauv-simulator-1')
   )


   return LaunchDescription([
      target_name,
      Node(
         package='neptus_interface',
         executable='neptus2unavsim',
         namespace='lauv_simulator_1',
         name='neptus2unavsim',
         parameters=[{
            'target_name': LaunchConfiguration('target_name')
         }]
      ),
      # IncludeLaunchDescription(
      # PythonLaunchDescriptionSource([os.path.join(
      #    get_package_share_directory('imcpy_ros_bridge'), 'launch'),
      #    '/lauv_simulator_1.launch.py'])
      # )
   ])
