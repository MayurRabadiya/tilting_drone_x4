#ifndef CONTROLLER_CONTROLLER_NODE_H
#define CONTROLLER_CONTROLLER_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <Eigen/Dense>

#include <string>


#include <chrono>
using namespace std::chrono_literals;

using std::placeholders::_1;

class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode();
    // virtual ~controller_node();
    void updateControllerOutput();

private:
    

    // Timers
    rclcpp::TimerBase::SharedPtr controllerTimer;
    rclcpp::TimerBase::SharedPtr offboardTimer;

    // subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_sub_;
    rclcpp::Subscription<px4_msgs::msg::ActuatorServos>::SharedPtr actuator_servos_sub_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr command_pose_sub_;

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr actuator_motors_publisher_;
    rclcpp::Publisher<px4_msgs::msg::ActuatorServos>::SharedPtr actuator_servos_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    // Services
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    // Messages
    px4_msgs::msg::ActuatorMotors actuator_motors_msg;


    // Controller gains
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    Eigen::Vector3d ang_vel_gain_;

    Eigen::Vector4d actuator_thrust_W;
    Eigen::Vector4d alpha_angle_W;

    px4_msgs::msg::VehicleStatus current_status_;
    bool connected_ = false;

    void loadParams();
    void arm();
    void disarm();

    // CallBacks
    void commandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg);
    // void commandTrajectoryCallback(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr& traj_msg);
    void vehicle_odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg);
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg);
    void actuator_motorsCallback(const px4_msgs::msg::ActuatorMotors::SharedPtr motor_msg);
    void actuator_servosCallback(const px4_msgs::msg::ActuatorServos::SharedPtr servo_msg);

    void publishActuatorMotorsMsg(const Eigen::VectorXd &alpha_angle, const Eigen::VectorXd &throttles);
    void publishOffboardControlModeMsg();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);


    inline Eigen::Vector3d rotateVectorFromToENU_NED(const Eigen::Vector3d &vec_in)
    {
        // NED (X North, Y East, Z Down) & ENU (X East, Y North, Z Up)
        Eigen::Vector3d vec_out;
        vec_out << vec_in[1], vec_in[0], -vec_in[2];
        return vec_out;
    }

    inline Eigen::Vector3d rotateVectorFromToFRD_FLU(const Eigen::Vector3d &vec_in)
    {
        // FRD (X Forward, Y Right, Z Down) & FLU (X Forward, Y Left, Z Up)
        Eigen::Vector3d vec_out;
        vec_out << vec_in[0], -vec_in[1], -vec_in[2];
        return vec_out;
    }

    inline Eigen::Quaterniond rotateQuaternionFromToENU_NED(const Eigen::Quaterniond &quat_in)
    {
        // Transform from orientation represented in ROS format to PX4 format and back
        //  * Two steps conversion:
        //  * 1. aircraft to NED is converted to aircraft to ENU (NED_to_ENU conversion)
        //  * 2. aircraft to ENU is converted to baselink to ENU (baselink_to_aircraft conversion)
        // OR
        //  * 1. baselink to ENU is converted to baselink to NED (ENU_to_NED conversion)
        //  * 2. baselink to NED is converted to aircraft to NED (aircraft_to_baselink conversion
        // NED_ENU_Q Static quaternion needed for rotating between ENU and NED frames
        Eigen::Vector3d euler_1(M_PI, 0.0, M_PI_2);
        Eigen::Quaterniond NED_ENU_Q(Eigen::AngleAxisd(euler_1.z(), Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(euler_1.y(), Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(euler_1.x(), Eigen::Vector3d::UnitX()));

        // AIRCRAFT_BASELINK_Q Static quaternion needed for rotating between aircraft and base_link frames
        Eigen::Vector3d euler_2(M_PI, 0.0, 0.0);
        Eigen::Quaterniond AIRCRAFT_BASELINK_Q(Eigen::AngleAxisd(euler_2.z(), Eigen::Vector3d::UnitZ()) *
                                               Eigen::AngleAxisd(euler_2.y(), Eigen::Vector3d::UnitY()) *
                                               Eigen::AngleAxisd(euler_2.x(), Eigen::Vector3d::UnitX()));

        return (NED_ENU_Q * quat_in) * AIRCRAFT_BASELINK_Q;
    }

    inline void eigenOdometryFromPX4Msg(const px4_msgs::msg::VehicleOdometry::SharedPtr msg,
                                        Eigen::Vector3d &position_W, Eigen::Quaterniond &orientation_B_W,
                                        Eigen::Vector3d &velocity_B, Eigen::Vector3d &angular_velocity_B)
    {
        position_W = rotateVectorFromToENU_NED(Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]));

        Eigen::Quaterniond quaternion(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        orientation_B_W = rotateQuaternionFromToENU_NED(quaternion);

        velocity_B = rotateVectorFromToENU_NED(Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]));

        angular_velocity_B = rotateVectorFromToFRD_FLU(Eigen::Vector3d(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]));
    }

   

    inline void eigenTrajectoryPointFromPoseMsg(
        const geometry_msgs::msg::PoseStamped::SharedPtr &msg, Eigen::Vector3d &position_W, Eigen::Quaterniond &orientation_W_B)
    {
        position_W << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        Eigen::Quaterniond quaternion(msg->pose.orientation.w,
                                      msg->pose.orientation.x,
                                      msg->pose.orientation.y,
                                      msg->pose.orientation.z);
        orientation_W_B = quaternion;
    }
};

#endif // px4_offboard_lowlevel_CONTROLLER_NODE_H
