#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

# Define the drone_x4_px4 path under the workspace directory
home_dir = os.path.expanduser('~')
px4_autopilot_dir = os.path.join(home_dir, 'Downloads/workspace', 'drone_x4_px4')
package_dir = get_package_share_directory('tilting_drone_x4')

def generate_launch_description():
    # Declare the drone type launch argument
    drone_type = LaunchConfiguration(
        'drone_type', default='gz_tilting_drone_x4')
    drone_type_args = DeclareLaunchArgument('drone_type', default_value=drone_type,
                                            description='Type of the drone to launch (gz_tilting_drone_x4, gz_x500)')

    # Execute the Micro XRCE-DDS Agent process
    run_microxrce_agent = ExecuteProcess(
        cmd=[['MicroXRCEAgent udp4 --port 8888 -v']],
        shell=True
    )

    # Execute the PX4 SITL (Software In The Loop) simulation command
    run_px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', drone_type],
        cwd=px4_autopilot_dir,
        output='screen'
    )

    # Define the visualizer node
    visualizer = Node(
        package='tilting_drone_x4',
        namespace='tilting_drone_x4',
        executable='visualizer.py',
        name='visualizer',
        output='screen'
    )

    # Define the offboard node
    offboard = Node(
        package='tilting_drone_x4',
        namespace='tilting_drone_x4',
        executable='offboard_control.py',
        name='Drone_X4_Node',
        output='screen'
    )

    # Define the RViz node for visualization
    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(package_dir, 'rviz', 'visualize.rviz')],
        output='screen'
    )

    # Create the launch description and populate it with actions
    ld = LaunchDescription([
        # Declare the drone type argument
        run_microxrce_agent,
        drone_type_args,
        run_px4_sitl,
        visualizer,
        rviz,
        offboard
    ])

    return ld

if __name__ == '__main__':
    generate_launch_description()
