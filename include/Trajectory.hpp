#pragma once
#include <vector>

struct TrajectoryPoint {
    double time;
    double position;
    double velocity;
    double acceleration;
};

class Trajectory {
public:
    static std::vector<TrajectoryPoint> generateQuintic(double q0, double q1, double v0, double v1, double tf, int steps);
};
