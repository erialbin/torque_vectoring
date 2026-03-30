#pragma once

// =============================================================================
// config.hpp — edit all tunable parameters here.
//
// Each struct maps to one pipeline component. Pass the values to the
// component constructors as shown in the usage example at the bottom.
// =============================================================================

namespace tv {

// -----------------------------------------------------------------------------
// Vehicle physical properties — shared across all three layers
// -----------------------------------------------------------------------------
struct VehicleConfig {
    double mass_{230.0};             // total vehicle mass [kg]
    double cg_height_{0.30};         // centre-of-gravity height [m]
    double wheelbase_{1.55};         // front-to-rear axle distance [m]
    double l_r_{0.82};               // rear axle to CG [m]  →  l_f = wheelbase - l_r
    double trackwidth_front_{1.20};  // front track width [m]
    double trackwidth_rear_{1.20};   // rear track width [m]
    double front_tlltd_{0.52};       // front TLLTD fraction (0 = all rear, 1 = all front)
    double wheel_radius_{0.20};      // effective wheel radius [m]
    double mu_{1.6};                 // tyre-road friction coefficient
    double t_max_{230.0};            // per-wheel motor torque limit [Nm]
};

// -----------------------------------------------------------------------------
// Aerodynamic and rolling-resistance losses — SmcYawController Fx feedforward
// -----------------------------------------------------------------------------
struct AeroConfig {
    double rho_{1.225};         // air density [kg/m³]  (1.225 at sea level, 15 °C)
    double c_d_{1.2};           // drag coefficient
    double frontal_area_{0.9};  // frontal reference area [m²]
    double c_rr_{0.015};        // rolling resistance coefficient
};

// -----------------------------------------------------------------------------
// Driving Force Observer — KinematicForceEstimator
// -----------------------------------------------------------------------------
struct ForceEstimatorConfig {
    double j_w_{0.5};      // wheel + motor rotational inertia [kg·m²]
    double lpf_tau_{0.02}; // DFO low-pass filter time constant [s]
                           // smaller → faster but noisier Fx estimate
};

// -----------------------------------------------------------------------------
// Bicycle model — SmcYawController feedforward (steady-state yaw moment)
// -----------------------------------------------------------------------------
struct BicycleModelConfig {
    double c_f_{50000.0};  // front axle cornering stiffness [N/rad]
    double c_r_{55000.0};  // rear axle cornering stiffness [N/rad]
                           // typical FS values: 40 000–70 000 N/rad
};

// -----------------------------------------------------------------------------
// Super-twisting SMC — SmcYawController yaw moment feedback
// -----------------------------------------------------------------------------
struct SmcConfig {
    double s_lambda_{0.5};   // integral weight in σ = e_r + λ·∫e_r dt
                              // larger → more aggressive integral action
    double k1_{800.0};        // gain on −k1·√|σ|·sign(σ)  [Nm]
    double k2_{200.0};        // gain on −k2·sign(σ)        [Nm/s]
                              // rule of thumb: k2 < k1²/2 for finite-time convergence
};

// -----------------------------------------------------------------------------
// Longitudinal PI — SmcYawController Fx demand (TU Munich / KIT FSG approach)
//
//   Fx_total = (k_p·ev + k_i·∫ev dt) · mass / eta,   ev = vx_ref − vx
// -----------------------------------------------------------------------------
struct LongitudinalPiConfig {
    double k_p_{2.0};   // proportional gain [1/s]
    double k_i_{0.5};   // integral gain     [1/s²]
    double eta_{0.95};  // drivetrain efficiency
};

// -----------------------------------------------------------------------------
// QP weights — QpTorqueAllocator
//
//   Cost:  (Bu−v)ᵀ diag(w_fx,w_mz) (Bu−v)  +  w_reg·‖u‖²
//
//   w_mz >> w_fx   → prioritise yaw tracking over Fx accuracy (recommended for racing)
//   w_reg          → prevents torque spikes; too large softens yaw response
// -----------------------------------------------------------------------------
struct AllocatorConfig {
    double w_fx_{1.0};     // Fx tracking weight
    double w_mz_{10.0};    // Mz tracking weight
    double w_reg_{0.001};  // torque magnitude regularisation
};

// -----------------------------------------------------------------------------
// Top-level config — instantiate this, edit values, pass sub-structs to ctors.
//
// Usage example:
//
//   const tv::Config cfg;
//
//   tv::KinematicForceEstimator estimator(
//       cfg.vehicle_.mass_, cfg.vehicle_.cg_height_, cfg.vehicle_.wheelbase_,
//       cfg.vehicle_.l_r_, cfg.vehicle_.trackwidth_front_, cfg.vehicle_.front_tlltd_,
//       cfg.force_estimator_.j_w_, cfg.vehicle_.wheel_radius_,
//       cfg.force_estimator_.lpf_tau_);
//
//   tv::SmcYawController controller(
//       cfg.bicycle_.c_f_, cfg.bicycle_.c_r_,
//       cfg.vehicle_.wheelbase_ - cfg.vehicle_.l_r_,   // l_f
//       cfg.vehicle_.l_r_,
//       cfg.aero_.rho_, cfg.aero_.c_d_, cfg.aero_.frontal_area_, cfg.aero_.c_rr_,
//       cfg.smc_.s_lambda_, cfg.smc_.k1_, cfg.smc_.k2_,
//       cfg.vehicle_.mass_,
//       cfg.longitudinal_pi_.k_p_, cfg.longitudinal_pi_.k_i_, cfg.longitudinal_pi_.eta_);
//
//   tv::QpTorqueAllocator allocator(
//       cfg.vehicle_.wheel_radius_,
//       cfg.vehicle_.trackwidth_front_, cfg.vehicle_.trackwidth_rear_,
//       cfg.vehicle_.t_max_, cfg.vehicle_.mu_,
//       cfg.allocator_.w_fx_, cfg.allocator_.w_mz_, cfg.allocator_.w_reg_);
// -----------------------------------------------------------------------------
struct Config {
    VehicleConfig vehicle_{};
    AeroConfig aero_{};
    ForceEstimatorConfig force_estimator_{};
    BicycleModelConfig bicycle_{};
    SmcConfig smc_{};
    LongitudinalPiConfig longitudinal_pi_{};
    AllocatorConfig allocator_{};
};

}  // namespace tv
