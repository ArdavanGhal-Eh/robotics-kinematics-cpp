#include "Kinematics.hpp"
#include <cmath>

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

    for (int i = 0; i < 6; ++i) {
        T = T * getDHMatrix(alpha[i], a[i], d[i], joints[i]);
    }
    return T;
}
