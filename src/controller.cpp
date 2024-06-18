#include "../include/tilting_drone_x4/controller.h"

controller::controller()
{
}

void controller::calculateControllerOutput(Eigen::VectorXd *controller_torque_thrust)
{
        assert(controller_torque_thrust);

        controller_torque_thrust->resize(6);

        // Trajectory tracking.

        // Eigen::Vector3d p_d;
        // p_d << 0.0, 0.0, 3.0;

        // double d1, d2, d3, d4;

        // d1 = sqrt(pow((0-position_m_(0)), 2) + pow((2 - position_m_(1)), 2));
        // d2 = sqrt(pow((2-position_m_(0)), 2) + pow((2 - position_m_(1)), 2));
        // d3 = sqrt(pow((2-position_m_(0)), 2) + pow((0 - position_m_(1)), 2));
        // d4 = sqrt(pow((0-position_m_(0)), 2) + pow((0 - position_m_(1)), 2));

        // if (d1<0.2){p_d << 2, 2, 2;}
        // if (d2<0.2){p_d << 2, 0, 2;}
        // if (d3<0.2){p_d << 0, 0, 2;}
        // if (d4<0.2){p_d << 0, 2, 2;}



        // Eigen::Matrix3d r_d;
        // r_d = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
        //             * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
        //             * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());

        // Compute translational tracking errors.
        const Eigen::Vector3d e_p = position_m_ - position_d_;
        const Eigen::Vector3d r_p =  (-position_gain_.cwiseProduct(e_p) - velocity_gain_.cwiseProduct(velocity_m_));
        const Eigen::Vector3d r_p_g =  _gravity * Eigen::Vector3d::UnitZ() + r_p;
        Eigen::Vector3d thrust = R_m.transpose() * r_p_g *_uav_mass;



        const Eigen::Matrix3d e_R_matrix = 0.5 * (R_d.transpose() * R_m - R_m.transpose() * R_d);
        Eigen::Vector3d e_R;
        e_R << e_R_matrix(2, 1), e_R_matrix(0,2), e_R_matrix(1,0);
        const Eigen::Vector3d r_R = -attitude_gain_.cwiseProduct(r_R) - angular_rate_gain_.cwiseProduct(angular_velocity_m_);
        Eigen::Vector3d tau = _inertia_matrix.cwiseProduct(r_R);

        *controller_torque_thrust << thrust, tau;
}