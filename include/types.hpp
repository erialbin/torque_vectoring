#pragma once

#include <Eigen/Dense>

namespace tv {

// [vx (m/s), vy (m/s), yaw_rate (rad/s), steering_angle(rad)]
using VehicleState = Eigen::Vector4d;

// Per-wheel tire forces (8 components):
// [Fx_FL, Fx_FR, Fx_RL, Fx_RR, Fy_FL, Fy_FR, Fy_RL, Fy_RR, Fz_FL, Fz_FR, Fz_RL, Fz_RR]
// (N)
using ForceVector = Eigen::Matrix<double, 12, 1>;

// Net desired yaw moment (Nm)
using YawMoment = double;

// Per-wheel torque commands [FL, FR, RL, RR] (Nm)
using WheelTorques = Eigen::Vector4d;

}  // namespace tv
