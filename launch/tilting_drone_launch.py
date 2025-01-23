import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
#        Node(package='micro_ros_agent',
#             executable='micro_ros_agent',
#             output='screen',
#             arguments = ['udp4', '-p', '8888'],
#             ),
#        Node(package='px4_gui',
#             executable='px4_gui',
#             output='screen',
#             ),
#        Node(package='vrpn_px4_bridge',
#             executable='vrpn_px4_bridge',
#             output='screen',
#             parameters=[{
#                 'tracker_name':'sector5_a', #'sector5_2',
#                 'server':'192.168.1.20',
#                 }],
###             remappings=[('/fmu/in/vehicle_visual_odometry', '/fmu/in/VehicleVisualOdometry')],
#             ),
        Node(package='tilting_drone_x4',
            namespace='tilting_drone_x4',
            executable='offboard_control.py',
            name='Drone_X4_Node',
            output='screen'
            )
        ])
