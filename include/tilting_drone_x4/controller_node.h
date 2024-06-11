#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <string>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class controllerNode : public rclcpp::Node
{
public:
    controllerNode();

private:
    // Timers
    rclcpp::TimerBase::SharedPtr controllerTimer;
    rclcpp::TimerBase::SharedPtr offboardTimer;

    // subscribers
    // rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    // rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    // rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
    
    // Publishers
    // rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    
    // CallBacks
    // void commandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    // void commandTrajectoryCallback(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr& traj_msg);
    // void vehicle_odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    // void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg);

    // void publishActuatorMotorsMsg(const Eigen::VectorXd& throttles);
    // void publishThrustTorqueMsg(const Eigen::Vector4d& controller_output);
    // void publishAttitudeSetpointMsg(const Eigen::Vector4d& controller_output, const Eigen::Quaterniond& desired_quaternion);
    void publishOffboardControlModeMsg();
    void publish_vehicle_command(uint16_t command, float param1 =0.0, float param2 = 0.0);   

}

#endif // CONTROLLER_NODE_H