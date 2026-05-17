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
    static constexpr double mass{250.0};       // total vehicle mass [kg]
    static constexpr double cg_height{0.20};   // centre-of-gravity height [m]
    static constexpr double wheelbase{1.530};  // front-to-rear axle distance [m]
    static constexpr double l_r{0.82};         // rear axle to CG [m]  →  l_f = wheelbase - l_r
    static constexpr double trackwidth_front{1.20};  // front track width [m]
    static constexpr double trackwidth_rear{1.20};   // rear track width [m]
    static constexpr double front_tlltd{
        0.45};  // front TLLTD fraction (0 = all rear, 1 = all front)
    static constexpr double wheel_radius{0.20};  // effective wheel radius [m]
    static constexpr double mu{1.6};             // tyre-road friction coefficient
    static constexpr double t_max{230.0};        // per-wheel motor torque limit [Nm]
};

// -----------------------------------------------------------------------------
// Aerodynamic and rolling-resistance losses — SmcYawController Fx feedforward
// -----------------------------------------------------------------------------
struct AeroConfig {
    static constexpr double rho{1.225};         // air density [kg/m³]  (1.225 at sea level, 15 °C)
    static constexpr double c_d{1.2};           // drag coefficient
    static constexpr double frontal_area{0.9};  // frontal reference area [m²]
    static constexpr double c_rr{0.015};        // rolling resistance coefficient
};

// -----------------------------------------------------------------------------
// Driving Force Observer — KinematicForceEstimator
// -----------------------------------------------------------------------------
struct ForceEstimatorConfig {
    static constexpr double j_w_{0.5};       // wheel + motor rotational inertia [kg·m²]
    static constexpr double lpf_tau_{0.02};  // DFO low-pass filter time constant [s]
                                             // smaller → faster but noisier Fx estimate
};

// -----------------------------------------------------------------------------
// Bicycle model — SmcYawController feedforward (steady-state yaw moment)
// -----------------------------------------------------------------------------
struct BicycleModelConfig {
    static constexpr double c_f_{28000.0};  // front axle cornering stiffness [N/rad]
    static constexpr double c_r_{28000.0};  // rear axle cornering stiffness [N/rad]
                                            // typical FS values: 40 000–70 000 N/rad
};

// -----------------------------------------------------------------------------
// Super-twisting SMC — SmcYawController yaw moment feedback
// -----------------------------------------------------------------------------
struct SmcConfig {
    static constexpr double s_lambda_{0.5};  // integral weight in σ = e_r + λ·∫e_r dt
                                             // larger → more aggressive integral action
    static constexpr double k1_{800.0};      // gain on −k1·√|σ|·sign(σ)  [Nm]
    static constexpr double k2_{200.0};      // gain on −k2·sign(σ)        [Nm/s]
                                         // rule of thumb: k2 < k1²/2 for finite-time convergence
};

// -----------------------------------------------------------------------------
// Longitudinal PI — SmcYawController Fx demand (TU Munich / KIT FSG approach)
//
//   Fx_total = (k_p·ev + k_i·∫ev dt) · mass / eta,   ev = vx_ref − vx
// -----------------------------------------------------------------------------
struct LongitudinalPiConfig {
    static constexpr double k_p_{2.0};  // proportional gain [1/s]
    static constexpr double k_i_{0.0};  // integral gain     [1/s²]
    static constexpr double eta_{0.0};  // drivetrain efficiency
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
    static constexpr double w_fx_{1.0};     // Fx tracking weight
    static constexpr double w_mz_{10.0};    // Mz tracking weight
    static constexpr double w_reg_{0.001};  // torque magnitude regularisation
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
    static constexpr VehicleConfig vehicle_{};
    static constexpr AeroConfig aero_{};
    static constexpr ForceEstimatorConfig force_estimator_{};
    static constexpr BicycleModelConfig bicycle_{};
    static constexpr SmcConfig smc_{};
    static constexpr LongitudinalPiConfig longitudinal_pi_{};
    static constexpr AllocatorConfig allocator_{};
};

}  // namespace tv
