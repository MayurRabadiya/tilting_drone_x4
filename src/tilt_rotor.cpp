#include "tilting_drone_x4/tilt_rotor.h"

ControllerNode::ControllerNode()
    : Node("controller_node")
{
    loadParams();

    // Defining the compatible ROS 2 predefined QoS for PX4 topics
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Subscribers
    vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&ControllerNode::vehicle_odometryCallback, this, _1));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&ControllerNode::vehicleStatusCallback, this, _1));
    command_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("command/pose", 10, std::bind(&ControllerNode::commandPoseCallback, this, _1));
    actuator_motors_sub_ = this->create_subscription<px4_msgs::msg::ActuatorMotors>("/fmu/out/actuator_motors", qos, std::bind(&ControllerNode::actuator_motorsCallback, this, _1));
    actuator_servos_sub_ = this->create_subscription<px4_msgs::msg::ActuatorServos>("/fmu/out/actuator_servos", qos, std::bind(&ControllerNode::actuator_servosCallback, this, _1));

    // Publishers
    actuator_motors_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/fmu/in/actuator_motors", 10);
    actuator_servos_publisher_ = this->create_publisher<px4_msgs::msg::ActuatorServos>("/fmu/in/actuator_servos", 10);
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    // Parameters subscriber
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ControllerNode::parametersCallback, this, std::placeholders::_1));

    // Timers
    std::chrono::duration<double> offboard_period(0.33);
    std::chrono::duration<double> controller_period(0.01);
    offboardTimer = this->create_wall_timer(offboard_period, [=]()
                                            {publishOffboardControlModeMsg();
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        arm(); });
    controllerTimer = this->create_wall_timer(controller_period, [=]()
                                              { updateControllerOutput(); });
}

rcl_interfaces::msg::SetParametersResult ControllerNode::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // print info about the changed parameter
    for (const auto &param : parameters)
    {
        RCLCPP_INFO(this->get_logger(), "Parameter %s has changed to [%s]", param.get_name().c_str(), param.value_to_string().c_str());
    }
    return result;
}

void ControllerNode::loadParams()
{
    // Controller gains
    this->declare_parameter("control_gains.K_p_x", 0.0);
    this->declare_parameter("control_gains.K_p_y", 0.0);
    this->declare_parameter("control_gains.K_p_z", 0.0);
    this->declare_parameter("control_gains.K_v_x", 0.0);
    this->declare_parameter("control_gains.K_v_y", 0.0);
    this->declare_parameter("control_gains.K_v_z", 0.0);
    this->declare_parameter("control_gains.K_R_x", 0.0);
    this->declare_parameter("control_gains.K_R_y", 0.0);
    this->declare_parameter("control_gains.K_R_z", 0.0);
    this->declare_parameter("control_gains.K_w_x", 0.0);
    this->declare_parameter("control_gains.K_w_y", 0.0);
    this->declare_parameter("control_gains.K_w_z", 0.0);

    position_gain_ << this->get_parameter("control_gains.K_p_x").as_double(),
        this->get_parameter("control_gains.K_p_y").as_double(),
        this->get_parameter("control_gains.K_p_z").as_double();

    velocity_gain_ << this->get_parameter("control_gains.K_v_x").as_double(),
        this->get_parameter("control_gains.K_v_y").as_double(),
        this->get_parameter("control_gains.K_v_z").as_double();

    attitude_gain_ << this->get_parameter("control_gains.K_R_x").as_double(),
        this->get_parameter("control_gains.K_R_y").as_double(),
        this->get_parameter("control_gains.K_R_z").as_double();

    ang_vel_gain_ << this->get_parameter("control_gains.K_w_x").as_double(),
        this->get_parameter("control_gains.K_w_y").as_double(),
        this->get_parameter("control_gains.K_w_z").as_double();
}

void ControllerNode::arm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void ControllerNode::disarm()
{
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void ControllerNode::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void ControllerNode::publishOffboardControlModeMsg()
{
    px4_msgs::msg::OffboardControlMode offboard_msg{};
    offboard_msg.position = false;
    offboard_msg.velocity = false;
    offboard_msg.acceleration = false;
    offboard_msg.body_rate = false;
    offboard_msg.attitude = false;
    offboard_msg.thrust_and_torque = false;
    offboard_msg.direct_actuator = true;
    offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(offboard_msg);
    RCLCPP_INFO_ONCE(get_logger(), "Offboard enabled");
}

void ControllerNode::commandPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
{ // When a command is received
    // initialize vectors
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    eigenTrajectoryPointFromPoseMsg(pose_msg, position, orientation);
    RCLCPP_INFO_ONCE(get_logger(), "Controller got first command message.");
}

void ControllerNode::vehicle_odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr odom_msg)
{
    //  Debug message
    RCLCPP_INFO_ONCE(get_logger(), "Controller got first odometry message.");

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;

    eigenOdometryFromPX4Msg(odom_msg, position, orientation, velocity, angular_velocity);
}

void ControllerNode::actuator_motorsCallback(const px4_msgs::msg::ActuatorMotors::SharedPtr motor_msg)
{
    //  Debug message
    RCLCPP_INFO_ONCE(get_logger(), "Controller got first actuator motor message.");
    Eigen::Vector4d motor_thrust;
    motor_thrust << motor_msg->control[0], motor_msg->control[1], motor_msg->control[2], motor_msg->control[3];
    RCLCPP_INFO(this->get_logger(), "Motor: %f    %f    %f    %f ", motor_thrust[0], motor_thrust[1], motor_thrust[2], motor_thrust[3]);
}

void ControllerNode::actuator_servosCallback(const px4_msgs::msg::ActuatorServos::SharedPtr servo_msg)
{
    //  Debug message
    RCLCPP_INFO_ONCE(get_logger(), "Controller got first actuator servo message.");
    Eigen::Vector4d alpha_angle_;
    alpha_angle_ << servo_msg->control[0], servo_msg->control[1], servo_msg->control[2], servo_msg->control[3];
    RCLCPP_INFO(this->get_logger(), "Servo: %f    %f    %f    %f ", alpha_angle_[0], alpha_angle_[1], alpha_angle_[2], alpha_angle_[3]);
}

void ControllerNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg)
{
    current_status_ = *status_msg;
}

void ControllerNode::publishActuatorMotorsMsg(const Eigen::VectorXd &throttles, const Eigen::VectorXd &alpha_angle)
{
    // Lockstep should be disabled from PX4 and from the model.sdf file
    // direct motor throttles control
    // Prepare msg
    px4_msgs::msg::ActuatorMotors actuator_motors_msg;
    actuator_motors_msg.control = {(float)throttles[0], (float)throttles[1], (float)throttles[2], (float)throttles[3],
                                   std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                                   std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1")};
    actuator_motors_msg.reversible_flags = 0;
    actuator_motors_msg.timestamp = this->get_clock()->make_shared()->now().nanoseconds() / 1000;
    actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp;

    actuator_motors_msg.control = {(float)0.11, (float)0.11, (float)0.11, (float)0.11,
                                   std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                                   std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1")};

    actuator_motors_publisher_->publish(actuator_motors_msg);

    px4_msgs::msg::ActuatorServos actuator_servos_msg;
    actuator_servos_msg.timestamp = this->get_clock()->make_shared()->now().nanoseconds() / 1000;
    // actuator_servos_msg.control = {(float)alpha_angle[0], (float)alpha_angle[1], (float)alpha_angle[2], (float)alpha_angle[3], 0.0, 0.0, 0.0, 0.0};
    actuator_servos_msg.control = {0.1, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0};
    //actuator_servos_publisher_->publish(actuator_servos_msg);

    // RCLCPP_INFO(this->get_logger(), "Servo: %f    %f    %f    %f ", alpha_angle[0], alpha_angle[1], alpha_angle[2], alpha_angle[3]);
    // RCLCPP_INFO(this->get_logger(), "Motor: %f    %f    %f    %f ", alpha_angle_W[0], alpha_angle_W[1], alpha_angle_W[2], alpha_angle_W[3]);
}

void ControllerNode::updateControllerOutput()
{
    if (current_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
        publishActuatorMotorsMsg(actuator_thrust_W, alpha_angle_W);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ControllerNode>());

    rclcpp::shutdown();

    return 0;
}
