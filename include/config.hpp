#pragma once
#include <vehicle_params_cpp/dev19.hpp>

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
    static constexpr double mass{kthfs::vehicle::dev19::weight_kg};  // total vehicle mass [kg]
    static constexpr double cg_height{
        kthfs::vehicle::dev19::cg_height_m};  // centre-of-gravity height [m]
    static constexpr double wheelbase{
        kthfs::vehicle::dev19::wheelbase_m};  // front-to-rear axle distance [m]
    static constexpr double l_r{0.82};        // rear axle to CG [m]  →  l_f = wheelbase - l_r
    static constexpr double trackwidth_front{
        kthfs::vehicle::dev19::track_width_m};  // front track width [m]
    static constexpr double trackwidth_rear{
        kthfs::vehicle::dev19::track_width_m};  // rear track width [m]
    static constexpr double front_tlltd{
        0.45};  // front TLLTD fraction (0 = all rear, 1 = all front)
    static constexpr double wheel_radius{
        kthfs::vehicle::dev19::tire_radius_m};  // effective wheel radius [m]
    static constexpr double mu{1.6};            // tyre-road friction coefficient
    static constexpr double t_max{230.0};       // per-wheel motor torque limit [Nm]
};

// -----------------------------------------------------------------------------
// Aerodynamic and rolling-resistance losses — SmcYawController Fx feedforward
// -----------------------------------------------------------------------------
struct AeroConfig {
    static constexpr double rho{1.225};  // air density [kg/m³]  (1.225 at sea level, 15 °C)
    static constexpr double c_d{kthfs::vehicle::dev19::drag_coefficient};  // drag coefficient
    static constexpr double frontal_area{
        kthfs::vehicle::dev19::frontal_area};  // frontal reference area [m²]
    static constexpr double c_rr =
        0.0 {kthfs::vehicle::dev19::rolling_resistance};  // rolling resistance coefficient
};

// -----------------------------------------------------------------------------
// Driving Force Observer — KinematicForceEstimator
// -----------------------------------------------------------------------------
struct ForceEstimatorConfig {
    static constexpr double j_w{0.5};       // wheel + motor rotational inertia [kg·m²]
    static constexpr double lpf_tau{0.02};  // DFO low-pass filter time constant [s]
                                            // smaller → faster but noisier Fx estimate
};

// -----------------------------------------------------------------------------
// Bicycle model — SmcYawController feedforward (steady-state yaw moment)
// -----------------------------------------------------------------------------
struct BicycleModelConfig {
    static constexpr double c_f{
        kthfs::vehicle::dev19::cornerings_stiffness_n_rad};  // front axle cornering stiffness
                                                             // [N/rad]
    static constexpr double c_r{
        kthfs::vehicle::dev19::cornerings_stiffness_n_rad};  // rear axle cornering stiffness
                                                             // [N/rad] typical FS values: 40 000–70
                                                             // 000 N/rad
};

// -----------------------------------------------------------------------------
// Super-twisting SMC — SmcYawController yaw moment feedback
// -----------------------------------------------------------------------------
struct PIDYawControllerConfig {
    static constexpr double kp{0.5};  // Proportional term for yaw rate error tracking
    static constexpr double ki{0.1};  // Integral term for yaw rate error tracking
};

// -----------------------------------------------------------------------------
// Longitudinal PI — SmcYawController Fx demand (TU Munich / KIT FSG approach)
//
//   Fx_total = (k_p·ev + k_i·∫ev dt) · mass / eta,   ev = vx_ref − vx
// -----------------------------------------------------------------------------
struct LongitudinalPiConfig {
    static constexpr double k_p{2.0};  // proportional gain [1/s]
    static constexpr double k_i{0.0};  // integral gain     [1/s²]
    static constexpr double eta{0.0};  // drivetrain efficiency
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
    static constexpr double w_fx{1.0};     // Fx tracking weight
    static constexpr double w_mz{10.0};    // Mz tracking weight
    static constexpr double w_reg{0.001};  // torque magnitude regularisation
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
    static constexpr VehicleConfig vehicle{};
    static constexpr AeroConfig aero{};
    static constexpr ForceEstimatorConfig force_estimator{};
    static constexpr BicycleModelConfig bicycle{};
    static constexpr PIDYawControllerConfig pid_yaw{};
    static constexpr LongitudinalPiConfig longitudinal_pi{};
    static constexpr AllocatorConfig allocator{};
};

}  // namespace tv
