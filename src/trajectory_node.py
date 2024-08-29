#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped

from px4_msgs.msg import VehicleOdometry

from scipy.spatial.transform import Rotation as R
import numpy as np
import math


class TrajectoryGeneration(Node):
    """Node to generates different trajectory for UAV."""

    def __init__(self) -> None:
        super().__init__('Trajectory_Node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Create subscribers
        self.vehicle_odometry_subsciber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        # Create publishers
        self.drone_pose_publisher = self. create_publisher(
            PoseStamped, '/drone_x4/pose', 10)

        # Initialize variables
        self.theta = 0.0
        self.theta_2 = 1.0
        self.start = 0

        self.declare_parameter('radius',   5.0)
        self.declare_parameter('t_dt',   0.001)
        self.declare_parameter('mode',       0)
        self.declare_parameter('spiral_h', 0.1)

        self.declare_parameter('roll',  0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw',   0.0)

        self.declare_parameter('x_pos', 0.0)
        self.declare_parameter('y_pos', 0.0)
        self.declare_parameter('z_pos', 5.0)

        # Create a timer to publish control commands
        self.control_timer = self.create_timer(0.01, self.timer_callback)

    def vehicle_odometry_callback(self, odom_msgs):
        self.uav_position = odom_msgs.position
        self.uav_velocity = odom_msgs.velocity
        self.uav_orientation = odom_msgs.q
        self.uav_angular_velocity = odom_msgs.angular_velocity

    def trajectory_publish(self, position, orientation):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        pose_msg.pose.orientation.x = orientation[0]
        pose_msg.pose.orientation.y = orientation[1]
        pose_msg.pose.orientation.z = orientation[2]
        pose_msg.pose.orientation.w = orientation[3]
        self.drone_pose_publisher.publish(pose_msg)

    def parameter_callback(self):
        """Callback function to handle parameter changes."""
        self.radius = self.get_parameter('radius').value
        self.t_dt   = self.get_parameter('t_dt').value
        self.mode   = self.get_parameter('mode').value

        self.roll  = self.get_parameter('roll').value
        self.pitch = self.get_parameter('pitch').value
        self.yaw   = self.get_parameter('yaw').value

        self.x_pos = self.get_parameter('x_pos').value
        self.y_pos = self.get_parameter('y_pos').value
        self.z_pos = self.get_parameter('z_pos').value

    def mode_print(self):
        if self.mode == 0:
            self.get_logger().info("Switched to Manual mode...")
        elif self.mode == 1:
            self.get_logger().info("Switched to Infinity shape trajectory mode...")
        elif self.mode == 2:
            self.get_logger().info("Switched to Circular shape trajectory mode...")
        elif self.mode == 3:
            self.get_logger().info("Switched to Star shape trajectory mode...")

    def eular_to_quat(self, rpy):
        roll  = np.radians(rpy[0])
        pitch = np.radians(rpy[1])
        yaw   = np.radians(rpy[2])
        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        q = rot.as_quat()  # Quaternion [x, y, z, w]
        return q

    def frd_to_flu(self, frd):
        """Convert FRD coordinates to FLU coordinates."""
        # FRD (X Forward, Y Right, Z Down) & FLU (X Forward, Y Left, Z Up)
        flu = frd[0],  -frd[1], -frd[2]
        return flu

    def circular_traj(self):
        """Circular motion trajectory"""
        # Update position setpoint to follow circular trajectory
        x = self.radius * np.cos(self.theta)
        y = self.radius * np.sin(self.theta)
        z = self.z_pos

        # Initialize roll and pitch to zero
        roll = 0.0
        pitch = 0.0

        # Check if theta is within specified ranges for roll and pitch to change
        if (70 <= np.degrees(self.theta) <= 110) or \
        (160 <= np.degrees(self.theta) <= 200) or \
        (250 <= np.degrees(self.theta) <= 290) or \
        (340 <= np.degrees(self.theta) <= 360) or \
        (0 <= np.degrees(self.theta) <= 20):
            roll = np.degrees(np.arctan2(x, y))
            pitch = 90.0


        yaw = self.yaw
        print(np.degrees(self.theta))

        # Convert Euler angles to quaternion
        q = self.eular_to_quat([roll, pitch, yaw])

        # Publish the trajectory
        self.trajectory_publish([x, y, z], orientation=q)

        # Update theta for the next time step

        if np.degrees(self.theta) >= 360:
            self.theta = 0
        self.theta += self.t_dt
        # print("X: "+str(current_pos[0])+"  Y: "+str(current_pos[1])+"  Angle: "+str(self.theta))


    def spiral_traj(self):
        """Spiral motion trajectory"""
        # Update position setpoint to follow circular trajectory
        spiral_h = self.get_parameter('spiral_h').value
        x = self.radius * np.cos(self.theta)
        y = self.radius * np.sin(self.theta)
        z = self.z_pos + spiral_h * self.theta
        roll  = np.degrees(np.arctan2(x, y))
        pitch = 90.0
        yaw   = self.yaw
        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([x,  y, z], orientation = q)
        self.theta += self.t_dt

    def infinity_traj(self):
        """Infinity motion trajectory"""
        # Update position setpoint to follow infinity (lemniscate) trajectory
        a = self.radius
        x = a * np.sin(self.theta)
        y = a * np.sin(self.theta) * np.cos(self.theta)
        z = self.z_pos
        roll  = np.degrees(np.arctan2(x, y))
        pitch = 90.0
        yaw   = self.yaw
        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([x,  y, z], orientation = q)
        self.theta += self.t_dt

    def star_traj(self):
        """Star trajectory"""
        outer_radius = 5
        inner_radius = 2
        num_points = 5  # number of outer points
        # since each outer point has an adjacent inner point
        total_vertices = num_points * 2
        angle_between_vertices = 360 / total_vertices  # in degrees

        # Generate the coordinates
        i = self.start
        angle_deg = i * angle_between_vertices
        rad = np.radians(angle_deg)
        radius = outer_radius if i % 2 == 0 else inner_radius

        x = radius * math.cos(rad)
        y = radius * math.sin(rad)
        z = self.z_pos

        roll = np.degrees(np.arctan2(x, y))
        pitch = 90.0
        yaw   = self.yaw

        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([x,  y, z], orientation = q)

        current_pos = self.frd_to_flu(self.uav_position)
        ref_dist = np.sqrt((x ** 2) + (y ** 2))
        curr_dist = np.sqrt((current_pos[0] ** 2) + (current_pos[1] ** 2))

        if (np.abs(ref_dist - curr_dist)) < 0.1:
            self.start += 1
            if self.start == total_vertices + 1:
                self.start = 1


    def vertical_star_traj(self):
        """Star trajectory"""
        outer_radius = 5
        inner_radius = 2
        num_points = 5  # number of outer points
        # since each outer point has an adjacent inner point
        total_vertices = num_points * 2
        angle_between_vertices = 360 / total_vertices  # in degrees

        # Generate the coordinates
        i = self.start
        angle_deg = i * angle_between_vertices
        rad = np.radians(angle_deg)
        radius = outer_radius if i % 2 == 0 else inner_radius

        x = self.x_pos
        y = radius * math.cos(rad)
        z = radius * math.sin(rad) + 10

        roll = np.degrees(np.arctan2(y, z))
        pitch = 90.0
        yaw   = self.yaw

        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([x,  y, z-0.2], orientation = q)

        current_pos = self.frd_to_flu(self.uav_position)
        ref_dist = np.sqrt((z ** 2) + (y ** 2))
        curr_dist = np.sqrt((current_pos[2] ** 2) + (current_pos[1] ** 2))

        if (np.abs(ref_dist - curr_dist)) < 0.2:
            self.start += 1
            if self.start == total_vertices + 1:
                self.start = 1


    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.parameter_callback()
        if self.mode == 1:
            self.infinity_traj()
        elif self.mode == 2:
            self.circular_traj()
        elif self.mode == 3:
            self.star_traj()
        elif self.mode == 4:
            self.spiral_traj()
        elif self.mode == 0:
            q = self.eular_to_quat([self.roll, self.pitch, self.yaw])
            self.trajectory_publish([self.x_pos, self.y_pos, self.z_pos], orientation = q)
        else:
            q = self.eular_to_quat([self.roll, self.pitch, self.yaw])
            self.trajectory_publish([self.x_pos, self.y_pos, self.z_pos], orientation = q)


def main(args=None) -> None:
    print('Starting Trajectory generattion node...')
    rclpy.init(args=args)
    trajectory_generation = TrajectoryGeneration()
    rclpy.spin(trajectory_generation)
    trajectory_generation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
