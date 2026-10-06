#pragma once
#include <vector>
#include <Eigen/Dense>

class Kinematics {
public:
    static Eigen::Matrix4d getDHMatrix(double alpha, double a, double d, double theta);
    static Eigen::Matrix4d forwardKinematics(const std::vector<double>& joints);
};
