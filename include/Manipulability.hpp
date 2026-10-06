#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>

class ManipulabilityAnalyzer {
public:
    // Computes Yoshikawa's Manipulability Index: w = sqrt(det(J * J^T))
    static double computeYoshikawaIndex(const Eigen::MatrixXd& J) {
        Eigen::MatrixXd JJt = J * J.transpose();
        double det = JJt.determinant();
        return (det > 0.0) ? std::sqrt(det) : 0.0;
    }

    // Computes Jacobian condition number: kappa = sigma_max / sigma_min
    static double computeConditionNumber(const Eigen::MatrixXd& J) {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
        auto singular_values = svd.singularValues();
        double s_max = singular_values(0);
        double s_min = singular_values(singular_values.size() - 1);
        return (s_min > 1e-6) ? (s_max / s_min) : 1e6;
    }

    // Damped Least Squares (DLS) Singularity-Robust Pseudo-Inverse:
    // J_dls = J^T * (J * J^T + lambda^2 * I)^(-1)
    static Eigen::MatrixXd computeDlsInverse(const Eigen::MatrixXd& J, double lambda_max = 0.05, double w_threshold = 0.02) {
        double w = computeYoshikawaIndex(J);
        double lambda_sq = 0.0;
        if (w < w_threshold) {
            double ratio = (1.0 - w / w_threshold);
            lambda_sq = lambda_max * lambda_max * ratio * ratio;
        }

        Eigen::MatrixXd JJt = J * J.transpose();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols());
        Eigen::MatrixXd damped_inv = (JJt + lambda_sq * I).inverse();
        return J.transpose() * damped_inv;
    }
};
