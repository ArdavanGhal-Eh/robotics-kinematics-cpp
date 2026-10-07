#include "Kinematics.hpp"
#include <cmath>

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

Eigen::Matrix4d Kinematics::getDHMatrix(double alpha, double a, double d, double theta) {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), a * cos(theta),
         sin(theta),  cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
         0,           sin(alpha),               cos(alpha),              d,
         0,           0,                        0,                       1;
    return T;
}

Eigen::Matrix4d Kinematics::forwardKinematics(const std::vector<double>& joints) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const double a[6] = {0.0, -0.425, -0.392, 0.0, 0.0, 0.0};
    const double d[6] = {0.163, 0.0, 0.0, 0.134, 0.100, 0.100};
    const double alpha[6] = {M_PI_2, 0.0, 0.0, M_PI_2, -M_PI_2, 0.0};

    for (size_t i = 0; i < 6 && i < joints.size(); ++i) {
        T = T * getDHMatrix(alpha[i], a[i], d[i], joints[i]);
    }
    return T;
}

Eigen::MatrixXd Kinematics::computeJacobian(const std::vector<double>& joints, double eps) {
    Eigen::MatrixXd J(3, joints.size());
    Eigen::Vector3d p0 = forwardKinematics(joints).block<3,1>(0,3);

    for (size_t i = 0; i < joints.size(); ++i) {
        std::vector<double> q_perturbed = joints;
        q_perturbed[i] += eps;
        Eigen::Vector3d p_pert = forwardKinematics(q_perturbed).block<3,1>(0,3);
        J.col(i) = (p_pert - p0) / eps;
    }
    return J;
}

bool Kinematics::inverseKinematicsDLS(
    const Eigen::Vector3d& target_position,
    std::vector<double>& q,
    int max_iter,
    double tol,
    double lambda
) {
    if (q.size() != 6) {
        q = std::vector<double>(6, 0.0);
    }

    for (int iter = 0; iter < max_iter; ++iter) {
        Eigen::Vector3d current_p = forwardKinematics(q).block<3,1>(0,3);
        Eigen::Vector3d err = target_position - current_p;

        if (err.norm() < tol) {
            return true;
        }

        Eigen::MatrixXd J = computeJacobian(q);
        // DLS: delta_q = J^T * (J * J^T + lambda^2 * I)^(-1) * err
        Eigen::Matrix3d JJt = J * J.transpose();
        Eigen::Matrix3d damped = JJt + (lambda * lambda) * Eigen::Matrix3d::Identity();
        Eigen::Vector3d delta_cartesian = damped.ldlt().solve(err);
        Eigen::VectorXd delta_q = J.transpose() * delta_cartesian;

        for (size_t i = 0; i < q.size(); ++i) {
            q[i] += delta_q(i);
        }
    }

    Eigen::Vector3d final_p = forwardKinematics(q).block<3,1>(0,3);
    return (target_position - final_p).norm() < tol;
}
