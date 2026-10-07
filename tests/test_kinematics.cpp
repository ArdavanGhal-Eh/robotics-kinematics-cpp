#include <iostream>
#include <cassert>
#include <cmath>
#include "Kinematics.hpp"
#include "Manipulability.hpp"
#include "Trajectory.hpp"

void test_forward_kinematics() {
    std::vector<double> joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Matrix4d pose = Kinematics::forwardKinematics(joints);
    Eigen::Vector3d pos = pose.block<3,1>(0,3);
    
    // Homogeneous transform bottom row must be [0, 0, 0, 1]
    assert(std::abs(pose(3,0)) < 1e-6);
    assert(std::abs(pose(3,1)) < 1e-6);
    assert(std::abs(pose(3,2)) < 1e-6);
    assert(std::abs(pose(3,3) - 1.0) < 1e-6);

    std::cout << "✅ [PASS] test_forward_kinematics | End-effector pos: (" 
              << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n";
}

void test_quintic_trajectory() {
    double q0 = 0.0;
    double q1 = 1.57;
    double v0 = 0.0;
    double v1 = 0.0;
    double tf = 2.0;
    int steps = 20;

    auto traj = Trajectory::generateQuintic(q0, q1, v0, v1, tf, steps);
    assert(traj.size() == steps + 1);

    // Initial boundary conditions
    assert(std::abs(traj.front().position - q0) < 1e-5);
    assert(std::abs(traj.front().velocity - v0) < 1e-5);
    assert(std::abs(traj.front().acceleration) < 1e-5);

    // Final boundary conditions
    assert(std::abs(traj.back().position - q1) < 1e-5);
    assert(std::abs(traj.back().velocity - v1) < 1e-5);
    assert(std::abs(traj.back().acceleration) < 1e-5);

    std::cout << "✅ [PASS] test_quintic_trajectory | Boundary conditions verified.\n";
}

void test_manipulability() {
    std::vector<double> joints = {0.0, -0.785, 1.047, 0.0, 0.523, 0.0};
    Eigen::MatrixXd J = Kinematics::computeJacobian(joints);
    double w = ManipulabilityAnalyzer::computeYoshikawaIndex(J);
    double cond = ManipulabilityAnalyzer::computeConditionNumber(J);

    assert(w >= 0.0);
    assert(cond >= 1.0);

    std::cout << "✅ [PASS] test_manipulability | Yoshikawa: " << w << ", Condition: " << cond << "\n";
}

void test_inverse_kinematics() {
    std::vector<double> target_joints = {0.1, -0.6, 0.9, 0.0, 0.4, 0.0};
    Eigen::Vector3d target_pos = Kinematics::forwardKinematics(target_joints).block<3,1>(0,3);

    std::vector<double> q_guess = {0.0, -0.5, 0.8, 0.0, 0.3, 0.0};
    bool converged = Kinematics::inverseKinematicsDLS(target_pos, q_guess, 150, 1e-3, 0.05);

    Eigen::Vector3d solved_pos = Kinematics::forwardKinematics(q_guess).block<3,1>(0,3);
    double pos_err = (target_pos - solved_pos).norm();

    assert(converged);
    assert(pos_err < 2e-3);

    std::cout << "✅ [PASS] test_inverse_kinematics | DLS converged with error: " << pos_err << " m\n";
}

int main() {
    std::cout << "=== Running Robotics Kinematics & Trajectory Unit Tests ===\n";
    test_forward_kinematics();
    test_quintic_trajectory();
    test_manipulability();
    test_inverse_kinematics();
    std::cout << "All unit tests passed successfully!\n";
    return 0;
}
