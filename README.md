# 🦾 6-DOF Industrial Robotic Arm Kinematics & Trajectory Engine

High-performance robotics library in **Modern C++20** using **Eigen3** for forward kinematics (Denavit-Hartenberg parametrization) and quintic polynomial jerk-limited trajectory generation.

## Features
- Forward Kinematics solver using standard DH parameters
- Smooth $C^2$-continuous Quintic polynomial trajectory generation (zero start/end jerks)
- Modern CMake build system

## Build & Run
```bash
mkdir build && cd build
cmake ..
cmake --build .
./robotics_kinematics
```

## Author
**Ardavan Ghal-Eh** | Sharif University of Technology
