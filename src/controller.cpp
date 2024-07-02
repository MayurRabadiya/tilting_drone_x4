#include "../include/tilting_drone_x4/controller.h"

controller::controller()
{
}

void controller::calculateControllerOutput(Eigen::VectorXd *controller_torque_thrust)
{
        assert(controller_torque_thrust);

        controller_torque_thrust->resize(6);

        // Compute position tracking errors.
        const Eigen::Vector3d e_p = position_m_ - position_d_;
        const Eigen::Vector3d r_p = (-position_gain_.cwiseProduct(e_p) - velocity_gain_.cwiseProduct(velocity_m_));
        const Eigen::Vector3d r_p_g = _gravity * Eigen::Vector3d::UnitZ() + r_p;
        Eigen::Vector3d thrust = R_m.transpose() * r_p_g * _uav_mass;

        // Compute rotational errors.
        
        // Rotation error from Rotation Matrix
       
        // const Eigen::Matrix3d e_R_matrix = 0.5 * (R_d.transpose() * R_m - R_m.transpose() * R_d);
        // Eigen::Vector3d e_R;
        // e_R << e_R_matrix(2, 1), e_R_matrix(0, 2), e_R_matrix(1, 0);
        // const Eigen::Vector3d r_R = -attitude_gain_.cwiseProduct(e_R) - angular_rate_gain_.cwiseProduct(angular_velocity_m_);
        // Eigen::Vector3d tau = _inertia_matrix.cwiseProduct(r_R);


        
        // Rotation error from Qaternion

        Eigen::Quaterniond q1 = Q_d.normalized();
	Eigen::Quaterniond q2 = Q_m.inverse().normalized();
	Eigen::Quaterniond q_err = q2 * q1;
	q_err.normalize();

        Eigen::Vector3d e_R;
        e_R = Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z()).cwiseProduct(e_R);
	e_R *= q_err.w() > 0.0f ? 1.0f : -1.0f;  // sign;

        const Eigen::Vector3d r_R = -e_R - angular_rate_gain_.cwiseProduct(angular_velocity_m_);
        Eigen::Vector3d tau = _inertia_matrix.cwiseProduct(r_R);

        const Eigen::Matrix3d e_R_matrix = q_err.matrix();






        _e_p = e_p;
        _r_p = r_p;
        _r_p_g = r_p_g;
        
        _e_R_matrix = e_R_matrix;
        _e_R = e_R;
        _r_R = r_R;

        *controller_torque_thrust << thrust, tau;
}
