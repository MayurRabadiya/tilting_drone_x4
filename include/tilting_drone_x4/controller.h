/****************************************************************************
 *
 *   Copyright (c) 2023, SMART Research Group, Saxion University of
 *   Applied Sciences.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef CONTROLLER_CONTROLLER_H
#define CONTROLLER_CONTROLLER_H

#include <eigen3/Eigen/Eigen>

class controller
{
public:
    controller();
    void calculateControllerOutput(Eigen::VectorXd *controller_torque_thrust);

    // Setters
    void setOdometry(const Eigen::Vector3d &position_m, const Eigen::Quaterniond &orientation_m, const Eigen::Vector3d &velocity_m, const Eigen::Vector3d &angular_velocity_m)
    {
        Q_m = orientation_m;
        orientation_m.normalized();
        R_m = orientation_m.matrix();
        position_m_ = position_m;
        velocity_m_ = velocity_m;
        angular_velocity_m_ = angular_velocity_m;
    }

    #include <cmath>

    void setDesiredPose(const Eigen::Vector3d &position_d, const Eigen::Vector3d &orientation_d)
    {
        position_d_ = position_d;

        // Convert degrees → radians
        Eigen::Vector3d orientation_rad = orientation_d.array() * M_PI / 180.0;

        R_d = eulerAnglesToRotationMatrix(orientation_rad(0),orientation_rad(1),orientation_rad(2)); // (roll, pitch, yaw)
        Q_d = eulerAnglesToQuaternion(orientation_rad(0),orientation_rad(1),orientation_rad(2)); // (roll, pitch, yaw)
    }

    // Not Used yet
    void setTrajectoryPoint(const Eigen::Vector3d &position_d, const Eigen::Quaterniond &orientation_d)
    {
        position_d_ = position_d;
        R_d = orientation_d.matrix();
    }

    // Not used yet
    void setArmAngle(const Eigen::Vector4d &alpha_angle)
    {
        alpha_angle_ = alpha_angle;
    }

    // Not used yet
    void setActuatorThrust(const Eigen::Vector4d &actuator_thrust)
    {
        actuator_thrust_ = actuator_thrust;
    }

    void setKPositionGain(const Eigen::Vector3d &PositionGain)
    {
        position_gain_ = PositionGain;
    }

    void setKVelocityGain(const Eigen::Vector3d &VelocityGain)
    {
        velocity_gain_ = VelocityGain;
    }

    void setKAttitudeGain(const Eigen::Vector3d &AttitudeGain)
    {
        attitude_gain_ = AttitudeGain;
    }

    void setKAngularRateGain(const Eigen::Vector3d AngularRateGain)
    {
        angular_rate_gain_ = AngularRateGain;
    }

    void setUavMass(double uavMass)
    {
        _uav_mass = uavMass;
    }

    void setInertiaMatrix(const Eigen::Vector3d &inertiaMatrix)
    {
        _inertia_matrix = inertiaMatrix;
    }

    void setGravity(double gravity)
    {
        _gravity = gravity;
    }

    // Function to convert roll, pitch, yaw to a rotation matrix using Eigen
    Eigen::Matrix3d eulerAnglesToRotationMatrix(double roll, double pitch, double yaw)
    {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
        q.normalized();
        Eigen::Matrix3d rotationMatrix = q.matrix();
        return rotationMatrix;
    }

    // Function to convert roll, pitch, yaw to a Quaternion using Eigen
    Eigen::Quaterniond eulerAnglesToQuaternion(double roll, double pitch, double yaw)
    {
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
        return q;
    }

    // Debuge Errors
    Eigen::Vector3d _e_p;
    Eigen::Vector3d _r_p;
    Eigen::Vector3d _r_p_g;
    Eigen::Vector3d _e_R;
    Eigen::Vector3d _r_R;
    Eigen::Matrix3d _e_R_matrix;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    // UAV Parameter
    double _uav_mass;
    Eigen::Vector3d _inertia_matrix;
    double _gravity;

    Eigen::Vector4d actuator_thrust_;
    Eigen::Vector4d alpha_angle_;

    // Controller Gains
    Eigen::Vector3d position_gain_;
    Eigen::Vector3d velocity_gain_;
    Eigen::Vector3d attitude_gain_;
    Eigen::Vector3d angular_rate_gain_;

    // Current states
    Eigen::Vector3d position_m_;
    Eigen::Vector3d velocity_m_;
    Eigen::Matrix3d R_m;
    Eigen::Vector3d angular_velocity_m_;

    // References (desired from user)
    Eigen::Vector3d position_d_;
    Eigen::Matrix3d R_d;

    Eigen::Quaterniond Q_m; // Measured Qaternion from Drone (Current Orientation)
    Eigen::Quaterniond Q_d; // desired Quaternion from user
};

#endif // CONTROLLER_CONTROLLER_H
