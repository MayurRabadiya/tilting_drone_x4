#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3

from px4_msgs.msg import VehicleOdometry

from scipy.spatial.transform import Rotation as R
import numpy as np
import math


class TrajectoryGeneration(Node):
    """Node to generates various trajectories for UAV."""

    def __init__(self) -> None:
        # Initialize the ROS2 node with the name 'Trajectory_Node'
        super().__init__('Trajectory_Node')

        # Configure the Quality of Service (QoS) profile for subscribers and publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Create subscribers
        self.vehicle_odometry_subsciber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)

        # Create publishers
        self.drone_pose_publisher = self. create_publisher(PoseStamped, '/drone_x4/pose', 10)
        self.reference_eular = self.create_publisher(Vector3, '/drone_x4/ref_rpy', 10)
        self.current_eular = self.create_publisher(Vector3, '/drone_x4/cur_rpy', 10)

        # Initialize variables
        self.theta = 0.0
        self.start = 0
        self.current_pos = [0.0, 0.0, 0.0]

        self.reverse = False


        self.declare_parameter('mode', 0)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.declare_parameter('x_pos', 0.0)
        self.declare_parameter('y_pos', 0.0)
        self.declare_parameter('z_pos', 5.0)

        self.declare_parameter('radius', 5.0)
        self.declare_parameter('t_dt', 0.001)
        self.declare_parameter('spiral_h', 0.1)
        self.declare_parameter('speed', 1.2)

        # Create a timer to call the control loop at regular intervals
        self.control_timer = self.create_timer(0.01, self.timer_callback)

    def vehicle_odometry_callback(self, odom_msgs):
        """Callback function for vehicle odometry subscriber."""
        self.uav_position = odom_msgs.position
        self.uav_velocity = odom_msgs.velocity
        self.uav_orientation = odom_msgs.q
        self.uav_angular_velocity = odom_msgs.angular_velocity

        # Convert quaternion to Euler angles
        quat = [odom_msgs.q[1], -odom_msgs.q[2], -odom_msgs.q[3], odom_msgs.q[0]]
        rot = R.from_quat(quat).as_matrix()
        eular = R.from_matrix(rot).as_euler('xyz', degrees=True)
        self.current_pos = self.frd_to_flu(self.uav_position)

        cur_msg = Vector3()
        cur_msg.x = eular[0]
        cur_msg.y = eular[1]
        cur_msg.z = eular[2]
        self.current_eular.publish(cur_msg)

    def trajectory_publish(self, position, orientation):
        """Publish the desired trajectory pose."""
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
        self.mode   = self.get_parameter('mode').value

        self.roll  = self.get_parameter('roll').value
        self.pitch = self.get_parameter('pitch').value
        self.yaw   = self.get_parameter('yaw').value

        self.x_pos = self.get_parameter('x_pos').value
        self.y_pos = self.get_parameter('y_pos').value
        self.z_pos = self.get_parameter('z_pos').value

        self.radius = self.get_parameter('radius').value
        self.t_dt   = self.get_parameter('t_dt').value
        self.speed  = self.get_parameter('speed').value
        self.spiral_h = self.get_parameter('spiral_h').value

    def eular_to_quat(self, rpy):
        """Convert Euler angles to quaternion."""
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
        """Generate and publish circular motion trajectory."""
        # Update position setpoint to follow circular trajectory
        x = self.radius * np.cos(self.theta)
        y = self.radius * np.sin(self.theta)
        z = self.z_pos

        roll = np.degrees(np.arctan2(x, y))
        pitch = self.pitch
        yaw = self.yaw

        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([x, y, z], orientation=q)

        if np.degrees(self.theta) >= 360:
            self.theta = 0
        self.theta += self.t_dt

        # Publish reference Euler angles
        ref_msg = Vector3()
        ref_msg.x = float(roll)
        ref_msg.y = float(pitch)
        ref_msg.z = float(yaw)
        self.reference_eular.publish(ref_msg)


    def taj_tracking(self):
        """Generate and publish half-circular motion trajectory."""
        # Update position setpoint to follow half-circular trajectory
        x = 13.0 * np.cos(self.theta)
        z = 49 + 13.0 * np.sin(self.theta)
        y = self.y_pos

        roll = self.roll
        pitch = np.degrees(np.arctan2(x, 13.0 * np.sin(self.theta))) 
        yaw = self.yaw

        # Convert Euler angles to quaternion for orientation
        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([x, y, z], orientation=q)

        # Update theta for half-circle movement
        if not self.reverse:  # Moving forward
            self.theta += self.t_dt
            if self.theta >= (np.pi + np.deg2rad(10)):  # When reaching the end of the half-circle (π radians or 180 degrees)
                self.reverse = True  # Start moving back
        else:  # Moving back to start
            self.theta -= self.t_dt
            if self.theta <= 0:  # When back at the starting position
                self.reverse = False  # Start moving forward again

        # Publish reference Euler angles
        ref_msg = Vector3()
        ref_msg.x = float(roll)
        ref_msg.y = float(pitch)
        ref_msg.z = float(yaw)
        self.reference_eular.publish(ref_msg)

    def narrow_window(self):
        """Generate and publish trajectory to pass UAV from narrow window."""
        x = self.radius * np.cos(self.theta)
        y = self.radius * np.sin(self.theta)
        z = self.z_pos

        roll = 0.0
        pitch = 0.0

        # Adjust roll and pitch based on theta range
        if (70 <= np.degrees(self.theta) <= 110) or \
        (160 <= np.degrees(self.theta) <= 200) or \
        (250 <= np.degrees(self.theta) <= 290) or \
        (340 <= np.degrees(self.theta) <= 360) or \
        (0 <= np.degrees(self.theta) <= 20):
            roll = np.degrees(np.arctan2(x, y))
            pitch = 90.0
            
        yaw = self.yaw

        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([x, y, z], orientation=q)

        if np.degrees(self.theta) >= 360:
            self.theta = 0
        self.theta += self.t_dt

        # Publish reference Euler angles
        ref_msg = Vector3()
        ref_msg.x = float(roll)
        ref_msg.y = float(pitch)
        ref_msg.z = float(yaw)
        self.reference_eular.publish(ref_msg)

    def spiral_traj(self):
        """Generate and publish spiral motion trajectory."""
        x = self.radius * np.cos(self.theta)
        y = self.radius * np.sin(self.theta)
        z = self.z_pos + self.spiral_h * self.theta

        roll  = np.degrees(np.arctan2(x, y))
        pitch = 90.0
        yaw   = self.yaw
        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([x,  y, z], orientation = q)
        self.theta += self.t_dt

    def lemniscate_traj(self):
        """Generate and publish lemniscate (infinity) motion trajectory."""
        x = self.radius * np.sin(self.theta)
        y = self.radius * np.sin(self.theta) * np.cos(self.theta)
        z = self.z_pos

        roll  = np.degrees(np.arctan2(x, y))
        pitch = 90.0
        yaw   = self.yaw

        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([x,  y, z], orientation = q)
        self.theta += self.t_dt

    def control_drone_speed(self, current_position, target_position):
        direction_vector = np.array(target_position) - np.array(current_position)
        distance_to_target = np.linalg.norm(direction_vector)

        if distance_to_target != 0:
            direction_unit_vector = direction_vector / distance_to_target
        else:
            direction_unit_vector = np.array([0, 0])

        if self.speed < distance_to_target:
            next_position = np.array(current_position) + direction_unit_vector * self.speed
        else:
            next_position = target_position

        return tuple(next_position)

    def star_traj(self):
        """Generate and publish star shape trajectory."""
        outer_radius = 10
        inner_radius = 5
        num_points   = 5  # number of outer points
        total_vertices = num_points * 2
        angle_between_vertices = 360 / total_vertices  # in degrees

        i = self.start
        angle_deg = i * angle_between_vertices
        rad = np.radians(angle_deg)
        radius = outer_radius if i % 2 == 0 else inner_radius

        x = radius * np.cos(rad)
        y = radius * np.sin(rad)
        z = self.z_pos

        pos_sp = self.control_drone_speed(self.current_pos, [x,y,z])
        roll = np.degrees(np.arctan2(x, y)) * 0
        pitch = 90.0 * 0
        yaw   = self.yaw

        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([pos_sp[0],  pos_sp[1], pos_sp[2]], orientation = q)

        self.current_pos = self.frd_to_flu(self.uav_position)
        ref_dist = np.sqrt((x ** 2) + (y ** 2))
        curr_dist = np.sqrt((self.current_pos[0] ** 2) + (self.current_pos[1] ** 2))

        if (np.abs(ref_dist - curr_dist)) < 0.1:
            self.start += 1
            if self.start == total_vertices + 1:
                self.start = 1

    def vertical_star_traj(self):
        """Generate and publish vertical star shape trajectory."""
        outer_radius = 20
        inner_radius = 3
        num_points = 3  # number of outer points
        # since each outer point has an adjacent inner point
        total_vertices = num_points * 2
        angle_between_vertices = 360 / total_vertices  # in degrees

        i = self.start
        angle_deg = i * angle_between_vertices
        rad = np.radians(angle_deg)
        radius = outer_radius if i % 2 == 0 else inner_radius

        x = self.x_pos
        y = radius * np.cos(rad)
        z = radius * np.sin(rad) + 17

        pos_sp = self.control_drone_speed(self.current_pos, [x,y,z])
        roll = np.degrees(np.arctan2(y, z))
        pitch = 90.0
        yaw   = self.yaw

        q = self.eular_to_quat([roll, pitch, yaw])
        self.trajectory_publish([pos_sp[0],  pos_sp[1], pos_sp[2]], orientation = q)

        ref_dist = np.sqrt((z ** 2) + (y ** 2))
        curr_dist = np.sqrt((self.current_pos[2] ** 2) + (self.current_pos[1] ** 2))

        if (np.abs(ref_dist - curr_dist)) < 0.3:
            self.start += 1
            if self.start == total_vertices + 1:
                self.start = 1

    def wind_turbine(self) :
        # positions = [ 
        #     (-3.5, -7.0,   25.0,    0,   0, 0),
        #     (-3.5,  4.0,   14.5,    20,  0, 0),
        #     (-3.5,  7.5,   15.0,    20,  0, 0),
        #     (-3.5, -2.0,   28.0,   -20,  0, 180),
        #     (-3.5, -3.0,   29.5,    30,  0, 180),
        #     (-3.5,  2.0,   45.0,    30,  0, 180),
        #     (-3.5, -2.0,   45.0,   -30,  0, 0),
        #     (-3.5, -7.0,   30.0,   -30,  0, 0),
        #     (-3.5, -8.0,   29.0,   -30,  0, 0),
        #     (-3.5, -25.0,  25.0,   -80, 0, 0),
        #     (-3.5, -25.0,  23.0,    110, 0, 0),
        #     (-3.5, -7.0,   24.0,    110, 0, 0)]


        positions = [ 
            (-3.5, -7.0,   25.0,    0,   0, 0),
            (-3.5,  4.0,   14.5,    0,  0, 0),
            (-3.5,  7.5,   15.0,    0,  0, 180),
            (-3.5, -2.0,   28.0,   -0,  0, 180),
            (-3.5, -3.0,   29.5,    0,  0, 180),
            (-3.5,  2.0,   45.0,    0,  0, 180),
            (-3.5, -2.0,   45.0,   -0,  0, 180),
            (-3.5, -7.0,   30.0,   -0,  0, 0),
            (-3.5, -8.0,   29.0,   -0,  0, 0),
            (-3.5, -25.0,  25.0,   -0, 0, 0),
            (-3.5, -25.0,  23.0,    0, 0, 0),
            (-3.5, -7.0,   24.0,    0, 0, 0)]



        point = positions[self.start]

        pos_sp = self.control_drone_speed(self.current_pos, [float(point[0]),float(point[1]),float(point[2])])

        q = self.eular_to_quat([point[3], point[4], point[5]])
        self.trajectory_publish([pos_sp[0],  pos_sp[1], pos_sp[2]], orientation = q)

        ref_dist = np.sqrt((point[0] ** 2) + (point[1] ** 2) + (point[2] ** 2))
        curr_dist = np.sqrt((self.current_pos[0] ** 2) + (self.current_pos[1] ** 2) + + (self.current_pos[2] ** 2))

        x_diff = abs((point[0]) - (self.current_pos[0]))
        y_diff = abs((point[1]) - (self.current_pos[1]))
        z_diff = abs((point[2]) - (self.current_pos[2]))
        print("Z: "+str(z_diff)+"    Y: "+str(y_diff))

        z_diff = z_diff - self.t_dt

        if  y_diff <= 0.08  and z_diff <= 0.08:
            self.start += 1
            if self.start == len(positions):
                self.start = 0


        # if (np.abs(ref_dist - curr_dist)) < 0.07:
        #     self.start += 1
        #     if self.start == len(positions):
        #         self.start = 0

        

    def manual_control(self):
        q = self.eular_to_quat([self.roll, self.pitch, self.yaw])
        self.trajectory_publish([self.x_pos, self.y_pos, self.z_pos], orientation = q)

    def manual_control_slow(self):
        q = self.eular_to_quat([self.roll, self.pitch, self.yaw])
        pos_sp = self.control_drone_speed(self.current_pos, [self.x_pos, self.y_pos, self.z_pos])
        self.trajectory_publish(pos_sp, orientation = q)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.parameter_callback()
        if self.mode   == 1: self.lemniscate_traj()
        elif self.mode == 2: self.circular_traj()
        elif self.mode == 3: self.star_traj()
        elif self.mode == 4: self.spiral_traj()
        elif self.mode == 5: self.wind_turbine()
        elif self.mode == 6: self.narrow_window()
        elif self.mode == 7: self.taj_tracking()
        elif self.mode == 0: self.manual_control()
        else: self.manual_control_slow()

def main(args=None):
    print('Starting Trajectory generation node...')
    print('Mode 0: manual_control')
    print('Mode 1: lemniscate_traj')
    print('Mode 2: circular_traj')
    print('Mode 3: star_traj')
    print('Mode 4: spiral_traj')
    print('Mode 5: wind_turbine')
    print('Mode 6: narrow_window')
    print('Mode 7: taj_tracking')

    rclpy.init(args=args)
    trajectory_generation = TrajectoryGeneration()
    try:
        rclpy.spin(trajectory_generation)
    except KeyboardInterrupt:
        trajectory_generation.get_logger().info('trajectory_generation stopped cleanly')
    except BaseException:
        import traceback
        trajectory_generation.get_logger().error('Exception in trajectory_generation: {}'.format(traceback.format_exc()))
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        trajectory_generation.destroy_node()


if __name__ == '__main__':
    main()
