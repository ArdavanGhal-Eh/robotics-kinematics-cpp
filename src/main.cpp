#include <iostream>
#include "Kinematics.hpp"
#include "Trajectory.hpp"

int main() {
    std::cout << "6-DOF Industrial Robotic Arm Kinematics & Trajectory Engine (C++20)" << std::endl;
    std::vector<double> jointAngles = {0.0, -M_PI/4, M_PI/3, 0.0, M_PI/6, 0.0};
    Eigen::Matrix4d pose = Kinematics::forwardKinematics(jointAngles);
    std::cout << "End Effector Position (X,Y,Z):\n" << pose.block<3,1>(0,3) << std::endl;

    auto trajectory = Trajectory::generateQuintic(0.0, 1.57, 0.0, 0.0, 2.0, 10);
    std::cout << "\nQuintic Trajectory Profile (10 waypoints):\n";
    for (const auto& pt : trajectory) {
        std::cout << "t=" << pt.time << "s | pos=" << pt.position << " rad | vel=" << pt.velocity << " rad/s\n";
    }
    return 0;
}
