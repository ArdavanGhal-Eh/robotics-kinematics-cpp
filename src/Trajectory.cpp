#include "Trajectory.hpp"

std::vector<TrajectoryPoint> Trajectory::generateQuintic(double q0, double q1, double v0, double v1, double tf, int steps) {
    std::vector<TrajectoryPoint> points;
    double a0 = q0;
    double a1 = v0;
    double a2 = 0.0;
    double a3 = (20.0 * (q1 - q0) - (8.0 * v1 + 12.0 * v0) * tf) / (2.0 * tf * tf * tf);
    double a4 = (30.0 * (q0 - q1) + (14.0 * v1 + 16.0 * v0) * tf) / (2.0 * tf * tf * tf * tf);
    double a5 = (12.0 * (q1 - q0) - 6.0 * (v1 + v0) * tf) / (2.0 * tf * tf * tf * tf * tf);

    for (int i = 0; i <= steps; ++i) {
        double t = tf * (double)i / steps;
        double pos = a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
        double vel = a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t;
        double acc = 2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t;
        points.push_back({t, pos, vel, acc});
    }
    return points;
}
