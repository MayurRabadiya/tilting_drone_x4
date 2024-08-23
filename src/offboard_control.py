#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import TiltingDroneX4AttitudeSetpoint
from px4_msgs.msg import TiltingDroneX4Gains

from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult

from scipy.spatial.transform import Rotation as R
import numpy as np
import math


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('Drone_X4_Node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_odometry_subsciber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.VehicleControlMode_pub = self.create_publisher(
            VehicleControlMode, '/fmu/in/vehicle_control_mode', qos_profile)
        self.tilting_drone_x4_attitude_setpoint_pub = self.create_publisher(
            TiltingDroneX4AttitudeSetpoint, '/fmu/in/tilting_drone_x4_attitude_setpoint', qos_profile)
        self.tilting_drone_x4_gains_pub = self.create_publisher(
            TiltingDroneX4Gains, '/fmu/in/tilting_drone_x4_gains', qos_profile)

        self.drone_local_pose_publisher = self. create_publisher(
            PoseStamped, '/drone_x4/pose', 10)

        self.reference_pose_publisher = self. create_publisher(
            PoseStamped, '/reference/pose', 10)

        # Initialize variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.theta = 0.0
        self.start = 0

        self.declare_parameter('radius', 5.0)
        self.declare_parameter('t_dt', 0.001)
        self.declare_parameter('mode', 0)

        self.declare_parameter('r', 0.0)
        self.declare_parameter('p', 0.0)
        self.declare_parameter('y', 0.0)

        self.declare_parameter('x_pos', 0.0)
        self.declare_parameter('y_pos', 0.0)
        self.declare_parameter('z_pos', 5.0)

        self.declare_parameter('rot_gain_x', 90.0)
        self.declare_parameter('rot_gain_y', 90.0)
        self.declare_parameter('rot_gain_z', 30.0)

        self.declare_parameter('angular_gain_x', 25.0)
        self.declare_parameter('angular_gain_y', 25.0)
        self.declare_parameter('angular_gain_z', 22.0)

        self.declare_parameter('angular_i_gain_x', 0.0)
        self.declare_parameter('angular_i_gain_y', 0.0)
        self.declare_parameter('angular_i_gain_z', 0.0)

        self.declare_parameter('pos_gain_x', 8.2)
        self.declare_parameter('pos_gain_y', 8.2)
        self.declare_parameter('pos_gain_z', 9.2)

        self.declare_parameter('val_gain_x', 9.5)
        self.declare_parameter('val_gain_y', 9.5)
        self.declare_parameter('val_gain_z', 9.5)

        self.declare_parameter('pos_i_gain_x', 0.0)
        self.declare_parameter('pos_i_gain_y', 0.0)
        self.declare_parameter('pos_i_gain_z', 0.0)



        # Create a timer to publish control commands
        self.offboard_timer = self.create_timer(0.02, self.offboard_callback)
        self.control_timer = self.create_timer(0.01, self.timer_callback)

        # Add parameter callback
        # self.add_on_set_parameters_callback(self.parameter_callback)

    def vehicle_odometry_callback(self, odom_msgs):
        self.position_w = odom_msgs.position
        self.velocity_w = odom_msgs.velocity
        self.orientation_b_w = R.from_quat([odom_msgs.q[0], odom_msgs.q[1],
                                            odom_msgs.q[2], odom_msgs.q[3]])
        self.angular_velocity_b = odom_msgs.angular_velocity

    def offboard_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        self.engage_offboard_mode()
        self.arm()

    def parameter_callback(self):
        """Callback function to handle parameter changes."""
        self.radius = self.get_parameter('radius').value
        self.t_dt = self.get_parameter('t_dt').value
        self.mode = self.get_parameter('mode').value

        self.r = self.get_parameter('r').value
        self.p = self.get_parameter('p').value
        self.y = self.get_parameter('y').value

        self.x_pos = self.get_parameter('x_pos').value
        self.y_pos = self.get_parameter('y_pos').value
        self.z_pos = self.get_parameter('z_pos').value

        self.rot_gain_x = self.get_parameter('rot_gain_x').value
        self.rot_gain_y = self.get_parameter('rot_gain_y').value
        self.rot_gain_z = self.get_parameter('rot_gain_z').value

        self.angular_gain_x = self.get_parameter('angular_gain_x').value
        self.angular_gain_y = self.get_parameter('angular_gain_y').value
        self.angular_gain_z = self.get_parameter('angular_gain_z').value

        self.angular_i_gain_x = self.get_parameter('angular_i_gain_x').value
        self.angular_i_gain_y = self.get_parameter('angular_i_gain_y').value
        self.angular_i_gain_z = self.get_parameter('angular_i_gain_z').value

        self.pos_gain_x = self.get_parameter('pos_gain_x').value
        self.pos_gain_y = self.get_parameter('pos_gain_y').value
        self.pos_gain_z = self.get_parameter('pos_gain_z').value

        self.val_gain_x = self.get_parameter('val_gain_x').value
        self.val_gain_y = self.get_parameter('val_gain_y').value
        self.val_gain_z = self.get_parameter('val_gain_z').value

        self.pos_i_gain_x = self.get_parameter('pos_i_gain_x').value
        self.pos_i_gain_y = self.get_parameter('pos_i_gain_y').value
        self.pos_i_gain_z = self.get_parameter('pos_i_gain_z').value


    def mode_print(self):
        if self.mode == 0:
            self.get_logger().info("Switched to Manual mode...")
        elif self.mode == 1:
            self.get_logger().info("Switched to Infinity shape trajectory mode...")
        elif self.mode == 2:
            self.get_logger().info("Switched to Circular shape trajectory mode...")
        elif self.mode == 3:
            self.get_logger().info("Switched to Star shape trajectory mode...")

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def arm(self):
        """Drone Arming"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

        vcm = VehicleControlMode()
        vcm.timestamp = int(Clock().now().nanoseconds / 1000)
        vcm.flag_control_manual_enabled = True
        vcm.flag_control_position_enabled = False
        vcm.flag_control_velocity_enabled = False
        vcm.flag_control_altitude_enabled = False
        self.VehicleControlMode_pub.publish(vcm)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        x_ned, y_ned, z_ned = self.frd_to_flu([x, y, z])
        msg.position = [x_ned, y_ned, z_ned]
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_gains(self):
        gain_msg = TiltingDroneX4Gains()
        gain_msg.rot_gain          = [self.rot_gain_x, self.rot_gain_y, self.rot_gain_z]
        gain_msg.angular_gain      = [self.angular_gain_x, self.angular_gain_y, self.angular_gain_z]
        gain_msg.rot_integral_gain = [self.angular_i_gain_x, self.angular_i_gain_y, self.angular_i_gain_z]

        gain_msg.pos_gain          = [self.pos_gain_x, self.pos_gain_y, self.pos_gain_z]
        gain_msg.val_gain          = [self.val_gain_x, self.val_gain_y, self.val_gain_z]
        gain_msg.pos_integral_gain = [self.pos_i_gain_x, self.pos_i_gain_y, self.pos_i_gain_z]

        gain_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.tilting_drone_x4_gains_pub.publish(gain_msg)

    def publish_attitude_setpoint(self, r: float, p: float, y: float):
        roll = np.radians(r)
        pitch = np.radians(p)
        yaw = np.radians(y)

        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        q = rot.as_quat()  # Quaternion [x, y, z, w]

        att_msg = TiltingDroneX4AttitudeSetpoint()
        att_msg.q = [q[3], q[0], -q[1], -q[2]]
        att_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.tilting_drone_x4_attitude_setpoint_pub.publish(att_msg)
        # self.get_logger().info(f"Publishing attitude setpoints {[r, p, y]}\n")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

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
        self.theta += self.t_dt

        angle = np.degrees(np.arctan2(x, y))
        self.publish_position_setpoint(x, y, z)
        self.publish_attitude_setpoint(angle, 90.0, 90.0)

    def infinity_traj(self):
        """Infinity motion trajectory"""
        # Update position setpoint to follow infinity (lemniscate) trajectory
        a = self.radius
        x = a * np.sin(self.theta)
        y = a * np.sin(self.theta) * np.cos(self.theta)
        z = self.z_pos
        self.theta += self.t_dt

        angle = np.degrees(np.arctan2(x, y))
        self.publish_position_setpoint(x, y, z)
        self.publish_attitude_setpoint(angle, 90.0, self.y)

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = -float(y)
        pose_msg.pose.position.z = -float(z)

        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 1.0
        pose_msg.pose.orientation.y = 1.0
        pose_msg.pose.orientation.z = 1.0
        self.drone_local_pose_publisher.publish(pose_msg)

    def star_traj(self):
        outer_radius = 5
        inner_radius = 2
        num_points = 3  # number of outer points
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
        angle = np.degrees(np.arctan2(x, y))

        self.publish_position_setpoint(x, y, z)
        self.publish_attitude_setpoint(angle, 90.0, self.y)

        current_pos = self.frd_to_flu(self.position_w)
        ref_dist = np.sqrt((x ** 2) + (y ** 2))
        curr_dist = np.sqrt((current_pos[0] ** 2) + (current_pos[1] ** 2))
        if (np.abs(ref_dist - curr_dist)) < 0.1:
            self.start += 1
            if self.start == total_vertices + 1:
                self.start = 1

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        self.parameter_callback()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_gains()
            if self.mode == 1:
                self.infinity_traj()
            elif self.mode == 2:
                self.circular_traj()
            elif self.mode == 3:
                self.star_traj()
            elif self.mode == 0:
                self.publish_position_setpoint(self.x_pos, self.y_pos, self.z_pos)
                self.publish_attitude_setpoint(self.r, self.p, self.y)
            else:
                self.publish_position_setpoint(self.x_pos, self.y_pos, self.z_pos)
                self.publish_attitude_setpoint(self.r, self.p, self.y)


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
