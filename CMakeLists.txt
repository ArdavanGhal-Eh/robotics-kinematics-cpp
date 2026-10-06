cmake_minimum_required(VERSION 3.16)
project(RoboticsKinematics VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(robotics_kinematics
    src/main.cpp
    src/Kinematics.cpp
    src/Trajectory.cpp
)
target_link_libraries(robotics_kinematics PRIVATE Eigen3::Eigen)
