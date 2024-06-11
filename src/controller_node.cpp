#include "tilting_drone_x4/controller_node.h"

ControllerNode::ControllerNode() : Node("controller_node")
{
    // Defining the compatible ROS 2 predefined QoS for PX4 topics
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    
    // Timers
    std::chrono::duration<double> offboard_period(0.33);
    std::chrono::duration<double> controller_period(0.01);
    offboardTimer = this->create_wall_timer(offboard_period, [=]()
                                            { publishOffboardControlModeMsg(); });
    // controllerTimer = this->create_wall_timer(controller_period, [=]()
    //                                           { updateControllerOutput(); });
}