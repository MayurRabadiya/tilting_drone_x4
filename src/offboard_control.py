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

from rcl_interfaces.msg import SetParametersResult

from scipy.spatial.transform import Rotation as R

import numpy as np

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
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_odometry_subsciber = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.VehicleControlMode_pub = self.create_publisher(VehicleControlMode, '/fmu/in/vehicle_control_mode', qos_profile)
        self.tilting_drone_x4_attitude_setpoint_pub = self.create_publisher(TiltingDroneX4AttitudeSetpoint, '/fmu/in/tilting_drone_x4_attitude_setpoint', qos_profile)

        # Initialize variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.theta = 0.0

        self.declare_parameter('radius', 5.0)
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('t_dt', 0.001)

        self.radius = self.get_parameter('radius').value
        self.altitude = self.get_parameter('altitude').value
        self.t_dt = self.get_parameter('t_dt').value

        self.declare_parameter('mode', 0)
        self.mode = self.get_parameter('mode').value

        self.declare_parameter('r', 0.0)
        self.declare_parameter('p', 0.0)
        self.declare_parameter('y', 0.0)

        self.r = self.get_parameter('r').value
        self.p = self.get_parameter('p').value
        self.y = self.get_parameter('y').value

        self.declare_parameter('x_pos', 0.0)
        self.declare_parameter('y_pos', 0.0)
        self.declare_parameter('z_pos', 5.0)

        self.x_pos = self.get_parameter('x_pos').value
        self.y_pos = self.get_parameter('y_pos').value
        self.z_pos = self.get_parameter('z_pos').value

        # Create a timer to publish control commands
        self.offboard_timer = self.create_timer(0.02, self.offboard_callback)
        self.control_timer = self.create_timer(0.01, self.timer_callback)

        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

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

    def parameter_callback(self, params):
        """Callback function to handle parameter changes."""
        for param in params:
            if param.name == 'altitude':
                self.altitude = param.value
            elif param.name == 'radius':
                self.radius = param.value
            elif param.name == 't_dt':
                self.t_dt = param.value
            elif param.name == 'r':
                self.r = param.value
            elif param.name == 'p':
                self.p = param.value
            elif param.name == 'y':
                self.y = param.value
            elif param.name == 'x_pos':
                self.x_pos = param.value
            elif param.name == 'y_pos':
                self.y_pos = param.value
            elif param.name == 'z_pos':
                self.z_pos = param.value
            elif param.name == 'mode':
                self.mode = param.value
                self.mode_print()
        return SetParametersResult(successful=True)

    def mode_print(self):
        if self.mode == 0: self.get_logger().info("Switched to Manual mode...")
        elif self.mode == 1: self.get_logger().info("Switched to Infinity shape trajectory mode...")
        elif self.mode == 2: self.get_logger().info("Switched to Circular shape trajectory mode...")

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
        msg.position = [x, y, z]
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_attitude_setpoint(self, r: float, p: float, y: float):
        roll = np.radians(self.r)
        pitch = np.radians(self.p)
        yaw = np.radians(self.y)

        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        q = rot.as_quat()  # Quaternion [x, y, z, w]

        att_msg = TiltingDroneX4AttitudeSetpoint()
        att_msg.q = [q[3], q[0], q[1], q[2]]
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

    def enu_to_ned(self, enu):
        """Convert ENU coordinates to NED coordinates."""
        # NED (X North, Y East, Z Down) & ENU (X East, Y North, Z Up)
        ned = enu[1],  enu[0], -enu[2]
        return ned

    def frd_to_flu(self, frd):
        """Convert FRD coordinates to FLU coordinates."""
        # FRD (X Forward, Y Right, Z Down) & FLU (X Forward, Y Left, Z Up)
        flu = frd[0],  -frd[1], -frd[2]
        return flu

    def rotate_quaternion_from_to_enu_ned(quat_in):
        # Transform from orientation represented in ROS format to PX4 format and back
        #  * Two steps conversion:
        #  * 1. aircraft to NED is converted to aircraft to ENU (NED_to_ENU conversion)
        #  * 2. aircraft to ENU is converted to baselink to ENU (baselink_to_aircraft conversion)
        # OR
        #  * 1. baselink to ENU is converted to baselink to NED (ENU_to_NED conversion)
        #  * 2. baselink to NED is converted to aircraft to NED (aircraft_to_baselink conversion
        # NED_ENU_Q Static quaternion needed for rotating between ENU and NED frames

        # Define the euler angles
        euler_1 = [np.pi, 0.0, np.pi / 2]
        euler_2 = [np.pi, 0.0, 0.0]

        # Convert euler angles to quaternions
        NED_ENU_Q = R.from_euler('zyx', euler_1).as_quat()
        AIRCRAFT_BASELINK_Q = R.from_euler('zyx', euler_2).as_quat()

        # Convert the input quaternion to a Rotation object
        quat_in = R.from_quat(quat_in)

        # Perform the quaternion multiplications
        NED_ENU_Q = R.from_quat(NED_ENU_Q)
        AIRCRAFT_BASELINK_Q = R.from_quat(AIRCRAFT_BASELINK_Q)

        # Convert the Rotation objects back to quaternions
        result_quat = ((NED_ENU_Q * quat_in) * AIRCRAFT_BASELINK_Q).as_quat()

        return result_quat

    def circular_traj(self):
        """Circular motion trajectory"""
        # Update position setpoint to follow circular trajectory

        x_enu = self.radius * np.cos(self.theta)
        y_enu = self.radius * np.sin(self.theta)
        z_enu = self.altitude
        self.theta += self.t_dt

        # Convert ENU --> NED    ENU_(X East, Y North, Z Up)_ROS / NED_(X North, Y East, Z Down)_PX4
        x_ned, y_ned, z_ned = self.enu_to_ned([x_enu, y_enu, z_enu])

        self.publish_position_setpoint(x_ned, y_ned, z_ned)
        self.publish_attitude_setpoint(self.r, self.p, self.y)

    def infinity_traj(self):
        """Infinity motion trajectory"""
        # Update position setpoint to follow infinity (lemniscate) trajectory
        a = self.radius
        x_enu = a * np.sin(self.theta)
        y_enu = a * np.sin(self.theta) * np.cos(self.theta)
        z_enu = self.altitude
        self.theta += self.t_dt

        # Convert ENU --> NED    ENU_(X East, Y North, Z Up)_ROS / NED_(X North, Y East, Z Down)_PX4
        x_ned, y_ned, z_ned = self.enu_to_ned([x_enu, y_enu, z_enu])

        angle = np.degrees(np.arctan2(x_ned, y_ned))
        self.publish_position_setpoint(x_ned, y_ned, z_ned)

        roll = np.radians(angle)
        pitch = np.radians(90)
        yaw = np.radians(self.y)

        rot = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        q = rot.as_quat()  # Quaternion [x, y, z, w]

        att_msg = TiltingDroneX4AttitudeSetpoint()
        att_msg.q = [q[3], q[0], q[1], q[2]]
        att_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.tilting_drone_x4_attitude_setpoint_pub.publish(att_msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.mode == 1: self.infinity_traj()
            elif self.mode == 2: self.circular_traj()
            elif self.mode == 0:
                self.publish_position_setpoint(self.x_pos, self.y_pos, -self.z_pos)
                self.publish_attitude_setpoint(self.r, self.p, self.y)
            else:
                self.publish_position_setpoint(self.x_pos, self.y_pos, -self.z_pos)
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
