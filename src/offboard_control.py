#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import TiltingDroneX4AttitudeSetpoint
from px4_msgs.msg import TiltingDroneX4Gains
from px4_msgs.msg import TiltingDroneX4TestParams

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

import numpy as np

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('Drone_X4_Offboard_Node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Create subscribers
        self.vehicle_odometry_subsciber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.trajectory_pose_subscriber = self.create_subscription(
            PoseStamped, '/drone_x4/pose', self.trajectory_pose_callback, 10)
        
        self.start_up_sub = self.create_subscription(
            Int32, '/drone_x4/start_up', self.start_up_callback, 10)
        

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
        self.tilting_drone_x4_test_params_pub = self.create_publisher(
            TiltingDroneX4TestParams, '/fmu/in/tilting_drone_x4_test_params', qos_profile)
        self.drone_local_pose_publisher = self. create_publisher(
            PoseStamped, '/drone_x4/local_pose', 10)

        # Initialize variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0

        self.start_up = 0

        self.pos_sp = np.array([0.0, 0.0, 0.0])
        self.orientation_sp = np.array([1.0, 0.0, 0.0, 0.0])
        self.setpoints_recived = False

        # Rotarion error gains
        self.declare_parameter('rot_gain_x', 60.0)
        self.declare_parameter('rot_gain_y', 60.0)
        self.declare_parameter('rot_gain_z', 30.0)

        # Angular velocity error gains
        self.declare_parameter('angular_gain_x', 25.0)
        self.declare_parameter('angular_gain_y', 25.0)
        self.declare_parameter('angular_gain_z', 22.0)

        # Angular velocity error integral gains
        self.declare_parameter('angular_i_gain_x', 0.0)
        self.declare_parameter('angular_i_gain_y', 0.0)
        self.declare_parameter('angular_i_gain_z', 0.0)

        # Position error gains
        self.declare_parameter('pos_gain_x', 3.2)
        self.declare_parameter('pos_gain_y', 3.2)
        self.declare_parameter('pos_gain_z', 9.2)

        # Velocity error gains
        self.declare_parameter('val_gain_x', 3.5)
        self.declare_parameter('val_gain_y', 3.5)
        self.declare_parameter('val_gain_z', 9.5)

        # Position error integral gains
        self.declare_parameter('pos_i_gain_x', 0.0)
        self.declare_parameter('pos_i_gain_y', 0.0)
        self.declare_parameter('pos_i_gain_z', 0.0)

        # Test Parameters
        self.declare_parameter('test_param1', 0.0)
        self.declare_parameter('test_param2', 0.0)
        self.declare_parameter('test_param3', 0.0)
        self.declare_parameter('test_param4', 0.0)


        # Create a timer to publish control commands
        self.offboard_timer = self.create_timer(0.02, self.offboard_callback)
        self.control_timer = self.create_timer(0.01, self.timer_callback)

    def offboard_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        self.engage_offboard_mode()
        if self.start_up == 1:
            self.arm()
        else:
            self.disarm()

    def start_up_callback(self, msg):
        self.start_up = msg.data


    def parameter_callback(self):
        """Callback function to handle parameter changes."""
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

        self.test_param_1 = self.get_parameter('test_param1').value
        self.test_param_2 = self.get_parameter('test_param2').value
        self.test_param_3 = self.get_parameter('test_param3').value
        self.test_param_4 = self.get_parameter('test_param4').value
        

    def mode_print(self):
        if self.mode == 0:
            self.get_logger().info("Switched to Manual mode...")
        elif self.mode == 1:
            self.get_logger().info("Switched to Infinity shape trajectory mode...")
        elif self.mode == 2:
            self.get_logger().info("Switched to Circular shape trajectory mode...")
        elif self.mode == 3:
            self.get_logger().info("Switched to Star shape trajectory mode...")

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

    def vehicle_odometry_callback(self, odom_msgs):
        self.uav_position = odom_msgs.position
        self.uav_orientation = odom_msgs.q
        pose_msgs = PoseStamped()
        flu_pos = self.frd_to_flu(self.uav_position)
        pose_msgs.pose.position.x = float(flu_pos[0])
        pose_msgs.pose.position.y = float(flu_pos[1])
        pose_msgs.pose.position.z = float(flu_pos[2])
        pose_msgs.pose.orientation.w = float( self.uav_orientation[0])
        pose_msgs.pose.orientation.x = float( self.uav_orientation[1])
        pose_msgs.pose.orientation.y = float(-self.uav_orientation[2])
        pose_msgs.pose.orientation.z = float(-self.uav_orientation[3])
        self.drone_local_pose_publisher.publish(pose_msgs)

    def trajectory_pose_callback(self, pose_msgs):
        """Subscibes the trajectory setpoint."""
        self.setpoints_recived = True
        self.pos_sp[0] = pose_msgs.pose.position.x
        self.pos_sp[1] = pose_msgs.pose.position.y
        self.pos_sp[2] = pose_msgs.pose.position.z
        self.orientation_sp[0] = pose_msgs.pose.orientation.w
        self.orientation_sp[1] = pose_msgs.pose.orientation.x
        self.orientation_sp[2] = pose_msgs.pose.orientation.y
        self.orientation_sp[3] = pose_msgs.pose.orientation.z

    def publish_test_params(self):
        """Publish Test Parameter values"""
        param_msg = TiltingDroneX4TestParams()
        param_msg.test_param = [self.test_param_1, self.test_param_2, self.test_param_3, self.test_param_4]
        param_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.tilting_drone_x4_test_params_pub.publish(param_msg)

    def publish_gains(self):
        """Publish the controller gains."""
        gain_msg = TiltingDroneX4Gains()
        gain_msg.rot_gain = [self.rot_gain_x, self.rot_gain_y, self.rot_gain_z]
        gain_msg.angular_gain = [self.angular_gain_x, self.angular_gain_y, self.angular_gain_z]
        gain_msg.rot_integral_gain = [self.angular_i_gain_x, self.angular_i_gain_y, self.angular_i_gain_z]

        gain_msg.pos_gain = [self.pos_gain_x, self.pos_gain_y, self.pos_gain_z]
        gain_msg.val_gain = [self.val_gain_x, self.val_gain_y, self.val_gain_z]
        gain_msg.pos_integral_gain = [self.pos_i_gain_x, self.pos_i_gain_y, self.pos_i_gain_z]

        gain_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.tilting_drone_x4_gains_pub.publish(gain_msg)

    def publish_position_setpoint(self):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        x_ned, y_ned, z_ned = self.frd_to_flu(self.pos_sp)
        msg.position = [x_ned, y_ned, z_ned]
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_attitude_setpoint(self):
        """Publish the attitude setpoint."""
        att_msg = TiltingDroneX4AttitudeSetpoint()
        att_msg.q = [self.orientation_sp[0], self.orientation_sp[1], -self.orientation_sp[2], -self.orientation_sp[3]]
        att_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.tilting_drone_x4_attitude_setpoint_pub.publish(att_msg)
        # self.get_logger().info(f"Publishing attitude setpoints {[r, p, y]}\n")

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        if (self.setpoints_recived):
            self.publish_offboard_control_heartbeat_signal()
            self.parameter_callback()
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_gains()
                self.publish_position_setpoint()
                self.publish_attitude_setpoint()
                self.publish_test_params()
        # else:
        #     self.get_logger().info('Waiting for trajectory setpoint...')


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
