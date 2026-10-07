#pragma once
#include <vector>
#include <Eigen/Dense>

class Kinematics {
public:
    static Eigen::Matrix4d getDHMatrix(double alpha, double a, double d, double theta);
    static Eigen::Matrix4d forwardKinematics(const std::vector<double>& joints);
    static Eigen::MatrixXd computeJacobian(const std::vector<double>& joints, double eps = 1e-6);
    static bool inverseKinematicsDLS(
        const Eigen::Vector3d& target_position,
        std::vector<double>& q,
        int max_iter = 100,
        double tol = 1e-3,
        double lambda = 0.05
    );
};
