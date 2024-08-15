#include "../include/tilting_drone_x4/controller.h"
#include "iostream"

controller::controller()
{
}

void controller::calculateControllerOutput(Eigen::VectorXd *controller_torque_thrust)
{
        assert(controller_torque_thrust);

        controller_torque_thrust->resize(6);

        // Compute position tracking errors.
        const Eigen::Vector3d e_p = position_m_ - position_d_;
        const Eigen::Vector3d r_p = (position_gain_.cwiseProduct(e_p) + velocity_gain_.cwiseProduct(velocity_m_)) + _uav_mass * _gravity * Eigen::Vector3d::UnitZ();
        Eigen::Vector3d thrust = R_m.transpose() * r_p;

        // Compute rotational errors.

        
        // const Eigen::Matrix3d e_R_matrix = 0.5 * (R_d.transpose() * R_m - R_m.transpose() * R_d);
        // Eigen::Vector3d e_R;
        // e_R << e_R_matrix(2, 1), e_R_matrix(0, 2), e_R_matrix(1, 0);
        // const Eigen::Vector3d r_R = -attitude_gain_.cwiseProduct(e_R) - angular_rate_gain_.cwiseProduct(angular_velocity_m_);
        // Eigen::Vector3d tau = _inertia_matrix.cwiseProduct(r_R);

        Eigen::Vector3d e_w = -angular_velocity_m_;
        Eigen::Matrix3d Psi = R_m.transpose() * R_d;

        double cos_theta = (Psi.trace() - 1) / 2;
        double theta = acosf(cos_theta);

        Eigen::Vector3d e_R;

        if (theta < 1e-6f)
        {
                // If theta is very small, return zero vector (no rotation)
                e_R.Zero();
        }
        else
        {
                double factor = theta / (2 * sin(theta));
                e_R(0) = factor * (Psi(2, 1) - Psi(1, 2));
                e_R(1) = factor * (Psi(0, 2) - Psi(2, 0));
                e_R(2) = factor * (Psi(1, 0) - Psi(0, 1));
        }

        Eigen::Vector3d tau = _inertia_matrix.cwiseProduct(e_R.cwiseProduct(attitude_gain_) + e_w.cwiseProduct(angular_rate_gain_));

        _e_p = position_m_;
        _r_p = r_p;
        // _r_p_g = r_p_g;

        _e_R_matrix = R_m;
        // _e_R = e_R;
        // _r_R = r_R;

        *controller_torque_thrust << thrust, tau;
}
