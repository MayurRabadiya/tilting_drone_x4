#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

# Define the PX4-Autopilot path under the workspace directory
home_dir = os.path.expanduser('~')
px4_autopilot_dir = os.path.join(home_dir, 'ws', 'PX4-Autopilot')
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
   #  run_px4_sitl = ExecuteProcess(
   #      cmd=['make', 'px4_sitl', drone_type],
   #      cwd=px4_autopilot_dir,
   #      output='screen'
   #  )

    # Define the visualizer node
    visualizer = Node(
        package='tilting_drone_x4',
        namespace='tilting_drone_x4',
        executable='visualizer.py',
        name='visualizer',
        output='screen'
    )

    offboard = Node(
        package='tilting_drone_x4',
        namespace='tilting_drone_x4',
        executable='offboard_control.py',
        name='offboard_control',
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
        drone_type_args,
        # Run the PX4 SITL process
        # run_px4_sitl,
        # Start the visualizer node
        visualizer,
        # Start the Micro XRCE-DDS Agent process
        run_microxrce_agent,
        # Start the RViz visualization tool
        rviz,
        offboard
    ])

    return ld

if __name__ == '__main__':
    generate_launch_description()
