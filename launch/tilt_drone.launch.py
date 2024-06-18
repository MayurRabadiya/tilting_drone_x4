import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config_1 = os.path.join(
      get_package_share_directory('tilting_drone_x4'),
      'config', 'uav_parameters',
      'x500_param.yaml'
      )
   
   config_3 = os.path.join(
      get_package_share_directory('tilting_drone_x4'),
      'config', 'controller',
      'initial_gains_x500.yaml'
      )
   
   return LaunchDescription([
      Node(
         package='tilting_drone_x4',
         executable='tilting_drone_x4_node',
         name='tilting_drone_x4',
         parameters=[config_1, config_3]
      )
   ])