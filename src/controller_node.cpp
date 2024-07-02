#include "tilting_drone_x4/controller_node.h"

ControllerNode::ControllerNode()
    : Node("controller_node")
{
    loadParams();
    // compute_ControlAllocation_and_ActuatorEffect_matrices();

    // Defining the compatible ROS 2 predefined QoS for PX4 topics
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Subscribers
    vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/out/vehicle_odometry", qos, std::bind(&ControllerNode::vehicle_odometryCallback, this, _1));
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("fmu/out/vehicle_status", qos, std::bind(&ControllerNode::vehicleStatusCallback, this, _1));
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
        if (param.get_name() == "control_mode")
        {
            control_mode_ = param.as_int();
        }
    }
    return result;
}

void ControllerNode::loadParams()
{
    // UAV Parameters
    this->declare_parameter("uav_parameters.mass", 0.0);
    this->declare_parameter("uav_parameters.arm_length", 0.0);
    this->declare_parameter("uav_parameters.num_of_arms", 4);
    this->declare_parameter("uav_parameters.input_scaling", 0);
    this->declare_parameter("uav_parameters.inertia.x", 0.0);
    this->declare_parameter("uav_parameters.inertia.y", 0.0);
    this->declare_parameter("uav_parameters.inertia.z", 0.0);
    this->declare_parameter("uav_parameters.gravity", 0.0);
    this->declare_parameter("uav_parameters.ServoEnable", true);

    servo_enable = this->get_parameter("uav_parameters.ServoEnable").as_bool();
    double _uav_mass = this->get_parameter("uav_parameters.mass").as_double();
    _arm_length = this->get_parameter("uav_parameters.arm_length").as_double();
    _num_of_arms = this->get_parameter("uav_parameters.num_of_arms").as_int();
    double _gravity = this->get_parameter("uav_parameters.gravity").as_double();
    _input_scaling = this->get_parameter("uav_parameters.input_scaling").as_int();
    double _inertia_x = this->get_parameter("uav_parameters.inertia.x").as_double();
    double _inertia_y = this->get_parameter("uav_parameters.inertia.y").as_double();
    double _inertia_z = this->get_parameter("uav_parameters.inertia.z").as_double();
    Eigen::Vector3d _inertia_matrix;
    _inertia_matrix << _inertia_x, _inertia_y, _inertia_z;

    // Load logic switches

    this->declare_parameter("control_mode", 1);
    control_mode_ = this->get_parameter("control_mode").as_int();

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

    this->declare_parameter("control_gains.position_x", 0.0);
    this->declare_parameter("control_gains.position_y", 0.0);
    this->declare_parameter("control_gains.position_z", 0.0);

    this->declare_parameter("control_gains.eular_roll", 0.0);
    this->declare_parameter("control_gains.eular_pitch", 0.0);
    this->declare_parameter("control_gains.eular_yaw", 0.0);

    position_des_ << this->get_parameter("control_gains.K_R_x").as_double(),
        this->get_parameter("control_gains.position_x").as_double(),
        this->get_parameter("control_gains.position_y").as_double();
    this->get_parameter("control_gains.position_z").as_double();

    eular_des_ << this->get_parameter("control_gains.K_w_x").as_double(),
        this->get_parameter("control_gains.eular_roll").as_double(),
        this->get_parameter("control_gains.eular_pitch").as_double();
    this->get_parameter("control_gains.eular_yaw").as_double();

    // pass the UAV Parameters and controller gains to the controller
    controller_.setUavMass(_uav_mass);
    controller_.setInertiaMatrix(_inertia_matrix);
    controller_.setGravity(_gravity);
    controller_.setKPositionGain(position_gain_);
    controller_.setKVelocityGain(velocity_gain_);
    controller_.setKAttitudeGain(attitude_gain_);
    controller_.setKAngularRateGain(ang_vel_gain_);
    controller_.setDesiredPose(position_des_, eular_des_);
}

void ControllerNode::px4InverseSITL(Eigen::VectorXd *alpha_angle, Eigen::VectorXd *throttles, const Eigen::VectorXd *wrench)
{
    Eigen::VectorXd omega;
    Eigen::VectorXd ones_temp;

    Eigen::MatrixXd mixing_matrix(8, 6);

    const double p = 0.70710678118; // sqrt(2)/2
    const double l = 0.18560;       // rotor arm length

    const double kt = 4.23e-06; // drage constant
    const double kf = 0.0026;   // force constant
    const double k = kt / kf;

    mixing_matrix << 1.0 / (4 * p), -1.0 / (4 * p), 0.0, 0.0, 0.0, -std::abs(l) * std::abs(l) / (4.0 * l * (std::abs(k) * std::abs(k) + std::abs(l) * std::abs(l))),
        k / (4.0 * l * p), -k / (4.0 * l * p), 1.0 / 4.0, 1.0 / (4.0 * l * p), -1.0 / (4.0 * l * p), -std::abs(k) * std::abs(k) / (4.0 * k * (std::abs(k) * std::abs(k) + std::abs(l) * std::abs(l))),
        1.0 / (4 * p), 1.0 / (4 * p), 0.0, 0.0, 0.0, -std::abs(l) * std::abs(l) / (4.0 * l * (std::abs(k) * std::abs(k) + std::abs(l) * std::abs(l))),
        -k / (4.0 * l * p), -k / (4.0 * l * p), 1.0 / 4.0, 1.0 / (4.0 * l * p), 1.0 / (4.0 * l * p), std::abs(k) * std::abs(k) / (4.0 * k * (std::abs(k) * std::abs(k) + std::abs(l) * std::abs(l))),
        -1.0 / (4 * p), 1.0 / (4 * p), 0.0, 0.0, 0.0, -std::abs(l) * std::abs(l) / (4.0 * l * (std::abs(k) * std::abs(k) + std::abs(l) * std::abs(l))),
        -k / (4.0 * l * p), k / (4.0 * l * p), 1.0 / 4.0, -1.0 / (4.0 * l * p), 1.0 / (4.0 * l * p), -std::abs(k) * std::abs(k) / (4.0 * k * (std::abs(k) * std::abs(k) + std::abs(l) * std::abs(l))),
        -1.0 / (4 * p), -1.0 / (4 * p), 0.0, 0.0, 0.0, -std::abs(l) * std::abs(l) / (4.0 * l * (std::abs(k) * std::abs(k) + std::abs(l) * std::abs(l))),
        k / (4.0 * l * p), k / (4.0 * l * p), 1.0 / 4.0, -1.0 / (4.0 * l * p), -1.0 / (4.0 * l * p), std::abs(k) * std::abs(k) / (4.0 * k * (std::abs(k) * std::abs(k) + std::abs(l) * std::abs(l)));

    Eigen::VectorXd result = mixing_matrix * (*wrench);
    Eigen::VectorXd t_T = *wrench;

    float T1 = sqrt(pow(result(0), 2) + pow(result(1), 2));
    float alpha1 = atan2(result(0), result(1));

    float T2 = sqrt(pow(result(2), 2) + pow(result(3), 2));
    float alpha2 = atan2(result(2), result(3));

    float T3 = sqrt(pow(result(4), 2) + pow(result(5), 2));
    float alpha3 = atan2(result(4), result(5));

    float T4 = sqrt(pow(result(6), 2) + pow(result(7), 2));
    float alpha4 = atan2(result(6), result(7));

    throttles->resize(4);
    *throttles << T1, T2, T3, T4;
    *throttles /= _input_scaling; // PX4 only accepts motor input in range 0 to 1

    alpha_angle->resize(4);

    *alpha_angle << alpha1, alpha2, alpha3, alpha4;
    *alpha_angle /= 3.1415926536; // PX4 only accepts servo motor input in range -1 to 1

    RCLCPP_INFO(this->get_logger(), "e_p    : %f    %f    %f", controller_._e_p[0], controller_._e_p[1], controller_._e_p[2]);
    RCLCPP_INFO(this->get_logger(), "r_p    : %f    %f    %f", controller_._r_p[0], controller_._r_p[1], controller_._r_p[2]);
    RCLCPP_INFO(this->get_logger(), "r_p_g  : %f    %f    %f", controller_._r_p_g[0], controller_._r_p_g[1], controller_._r_p_g[2]);
    RCLCPP_INFO(this->get_logger(), "e_R    : %f    %f    %f", controller_._e_R[0], controller_._e_R[1], controller_._e_R[2]);
    RCLCPP_INFO(this->get_logger(), "r_R    : %f    %f    %f", controller_._r_R[0], controller_._r_R[1], controller_._r_R[2]);
    RCLCPP_INFO(this->get_logger(), "e_R_matrix  :  \n%s", matrix3dToString(controller_._e_R_matrix).c_str());

    RCLCPP_INFO(this->get_logger(), "Thrust : %f    %f    %f", t_T[0], t_T[1], t_T[2]);
    RCLCPP_INFO(this->get_logger(), "Torque : %f    %f    %f", t_T[3], t_T[4], t_T[5]);
}

std::string ControllerNode::matrix3dToString(const Eigen::Matrix3d &matrix)
{
    std::ostringstream oss;
    for (int i = 0; i < 3; ++i)
    {
        oss << "[ ";
        for (int j = 0; j < 3; ++j)
        {
            oss << matrix(i, j) << " ";
        }
        oss << "]\n";
    }
    return oss.str();
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
    controller_.setTrajectoryPoint(position, orientation); // Send the command to controller_ obj
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

    controller_.setOdometry(position, orientation, velocity, angular_velocity);
}

void ControllerNode::actuator_motorsCallback(const px4_msgs::msg::ActuatorMotors::SharedPtr motor_msg)
{
    //  Debug message
    RCLCPP_INFO_ONCE(get_logger(), "Controller got first actuator motor message.");
    Eigen::Vector4d motor_thrust;
    motor_thrust << motor_msg->control[0], motor_msg->control[1], motor_msg->control[2], motor_msg->control[3];
    _setActuatorThrust(motor_thrust);
    // controller_.setActuatorThrust(motor_thrust);
}

void ControllerNode::actuator_servosCallback(const px4_msgs::msg::ActuatorServos::SharedPtr servo_msg)
{
    //  Debug message
    RCLCPP_INFO_ONCE(get_logger(), "Controller got first actuator servo message.");
    Eigen::Vector4d alpha_angle_;
    alpha_angle_ << servo_msg->control[0], servo_msg->control[1], servo_msg->control[2], servo_msg->control[3];
    _setArmAngle(alpha_angle_);
    // controller_.setArmAngle(alpha_angle);
}

void ControllerNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr status_msg)
{
    current_status_ = *status_msg;
}

void ControllerNode::publishActuatorMotorsMsg(const Eigen::VectorXd &throttles, const Eigen::VectorXd &alpha_angle)
{
    px4_msgs::msg::ActuatorMotors actuator_motors_msg;
    actuator_motors_msg.control = {(float)throttles[0], (float)throttles[1], (float)throttles[2], (float)throttles[3],
                                   std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1"),
                                   std::nanf("1"), std::nanf("1"), std::nanf("1"), std::nanf("1")};
    actuator_motors_msg.reversible_flags = 0;
    actuator_motors_msg.timestamp = this->get_clock()->make_shared()->now().nanoseconds() / 1000;
    actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp;

    actuator_motors_publisher_->publish(actuator_motors_msg);

    px4_msgs::msg::ActuatorServos actuator_servos_msg;
    actuator_servos_msg.timestamp = this->get_clock()->make_shared()->now().nanoseconds() / 1000;

    if (servo_enable)
    {
        actuator_servos_msg.control = {(float)alpha_angle[0], (float)alpha_angle[1], (float)alpha_angle[2], (float)alpha_angle[3], 0.0, 0.0, 0.0, 0.0};
    }
    else
    {
        actuator_servos_msg.control = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    actuator_servos_publisher_->publish(actuator_servos_msg);

    RCLCPP_INFO(this->get_logger(), "Servo : %f    %f    %f    %f ", alpha_angle[0], alpha_angle[1], alpha_angle[2], alpha_angle[3]);
    RCLCPP_INFO(this->get_logger(), "Motor : %f    %f    %f    %f \n", throttles[0], throttles[1], throttles[2], throttles[3]);
}

void ControllerNode::updateControllerOutput()
{
    //  calculate controller output
    Eigen::VectorXd controller_output;
    Eigen::Quaterniond desired_quaternion;
    controller_.calculateControllerOutput(&controller_output);

    // Normalize the controller output
    Eigen::Vector4d normalized_torque_thrust;
    Eigen::VectorXd throttles;
    Eigen::VectorXd alpha_angle;
    px4InverseSITL(&alpha_angle, &throttles, &controller_output);

    if (current_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {

        publishActuatorMotorsMsg(throttles, alpha_angle);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ControllerNode>());

    rclcpp::shutdown();

    return 0;
}