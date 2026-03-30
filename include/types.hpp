#pragma once

#include <array>

#include <Eigen/Dense>

namespace tv {

// Full vehicle state — populated from IMU, odometry, and steering sensor.
struct VehicleState {
    double vx_{0.0};             // longitudinal velocity [m/s]
    double vy_{0.0};             // lateral velocity      [m/s]
    double yaw_rate_{0.0};       // yaw rate              [rad/s]
    double steering_angle_{0.0}; // front wheel steer     [rad]
    double ax_{0.0};             // longitudinal accel    [m/s²]
    double ay_{0.0};             // lateral accel         [m/s²]
};

// Per-wheel tire forces, wheel order: FL, FR, RL, RR.
struct ForceVector {
    std::array<double, 4> fx_{};  // longitudinal forces [N]
    std::array<double, 4> fy_{};  // lateral forces      [N]
    std::array<double, 4> fz_{};  // normal forces       [N]
};

// High-level controller output: yaw moment + total longitudinal force demand.
struct YawMomentCommand {
    double mz_{0.0};        // corrective yaw moment [Nm]
    double fx_total_{0.0};  // total longitudinal force demand [N]
};

// Driver/planner inputs.
struct SteeringCommand {
    double steering_angle_{0.0};  // front wheel steer angle [rad]
    double vx_ref_{0.0};          // longitudinal velocity reference [m/s]
    double r_ref_{0.0};           // yaw rate reference [rad/s]
};

// Per-wheel torque commands [FL, FR, RL, RR] (Nm).
// Kept as Eigen — returned directly from the OSQP solver and used in DFO loops.
using WheelTorques = Eigen::Vector4d;

// Per-wheel angular velocities [FL, FR, RL, RR] (rad/s).
// Kept as Eigen — used in element-wise DFO loops.
using WheelOmegas = Eigen::Vector4d;

}  // namespace tv
