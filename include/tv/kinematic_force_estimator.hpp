#pragma once

#include <Eigen/Dense>

#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

class KinematicForceEstimator : public IForceEstimator<KinematicForceEstimator> {
   public:
    // Vehicle params:
    //   mass         [kg]
    //   cg_height    [m]
    //   wheelbase    [m]
    //   l_r          distance from rear axle to CG [m]
    //   trackwidth   [m]
    //   front_tlltd  front total lateral load transfer distribution fraction (0–1)
    // DFO params:
    //   j_w          wheel + motor rotational inertia [kg·m²]
    //   r_eff        effective wheel radius [m]
    //   lpf_tau      low-pass filter time constant [s]
    KinematicForceEstimator(double mass, double cg_height, double wheelbase, double l_r,
                            double trackwidth, double front_tlltd, double j_w, double r_eff,
                            double lpf_tau)
        : mass_(mass),
          cg_height_(cg_height),
          wheelbase_(wheelbase),
          l_r_(l_r),
          trackwidth_(trackwidth),
          front_tlltd_(front_tlltd),
          j_w_(j_w),
          r_eff_(r_eff),
          lpf_tau_(lpf_tau) {}

    // Feed per-wheel sensor data before each call to estimateForcesImpl.
    //   motor_torques   inverter torque demand [FL, FR, RL, RR] (Nm)
    //   wheel_omegas    wheel angular velocities [FL, FR, RL, RR] (rad/s)
    //   dt              time since last update (s)
    void updateWheelState(const WheelTorques& motor_torques, const WheelOmegas& wheel_omegas,
                          double dt) {
        prev_omegas_ = current_omegas_;
        current_omegas_ = wheel_omegas;
        motor_torques_ = motor_torques;
        dt_ = dt;
    }

    [[nodiscard]] ForceVector estimateForcesImpl(const VehicleState& s) {
        const double ax = s.ax_;
        const double ay = s.ay_;
        const double l_f = wheelbase_ - l_r_;

        ForceVector forces{};

        // --- Fx: Driving Force Observer (DFO) ---
        // F̂_x = (1/R_eff) * [T_motor − J_w·dω/dt], smoothed by first-order LPF.
        // Reference: Hori (2004), adopted by FS Team Tallinn (world #1 FSE 2024).
        if (dt_ > 0.0) {
            const double alpha = lpf_tau_ / (lpf_tau_ + dt_);
            for (int i = 0; i < 4; ++i) {
                const double d_omega_dt = (current_omegas_[i] - prev_omegas_[i]) / dt_;
                const double fx_raw = (motor_torques_[i] - j_w_ * d_omega_dt) / r_eff_;
                fx_filtered_[i] = alpha * fx_filtered_[i] + (1.0 - alpha) * fx_raw;
            }
        }
        for (int i = 0; i < 4; ++i) forces.fx_[i] = fx_filtered_[i];

        // --- Fz: quasi-static load transfer (AMZ acceleration-based method) ---
        // Uses IMU ax, ay directly — no kinematic approximation.
        // Reference: AMZ Racing (ETH Zurich), docs/tire_forces.md §1.
        const double fz_front_static = 0.5 * mass_ * 9.81 * l_r_ / wheelbase_;
        const double fz_rear_static = 0.5 * mass_ * 9.81 * l_f / wheelbase_;

        const double delta_fz_lat = mass_ * ay * cg_height_ / trackwidth_;
        const double delta_fz_lat_front = front_tlltd_ * delta_fz_lat;
        const double delta_fz_lat_rear = (1.0 - front_tlltd_) * delta_fz_lat;
        const double delta_fz_long = 0.5 * mass_ * ax * cg_height_ / wheelbase_;

        forces.fz_[0] = fz_front_static - delta_fz_long - delta_fz_lat_front;  // FL
        forces.fz_[1] = fz_front_static - delta_fz_long + delta_fz_lat_front;  // FR
        forces.fz_[2] = fz_rear_static + delta_fz_long - delta_fz_lat_rear;    // RL
        forces.fz_[3] = fz_rear_static + delta_fz_long + delta_fz_lat_rear;    // RR

        // --- Fy: steady-state bicycle model (using IMU ay) ---
        const double fy_f = 0.5 * (l_r_ / wheelbase_) * mass_ * ay;
        const double fy_r = 0.5 * (l_f / wheelbase_) * mass_ * ay;

        forces.fy_[0] = fy_f;  // FL
        forces.fy_[1] = fy_f;  // FR
        forces.fy_[2] = fy_r;  // RL
        forces.fy_[3] = fy_r;  // RR

        return forces;
    }

   private:
    // Vehicle parameters
    double mass_;
    double cg_height_;
    double wheelbase_;
    double l_r_;
    double trackwidth_;
    double front_tlltd_;  // front TLLTD fraction, e.g. 0.52

    // DFO parameters
    double j_w_;      // wheel + motor inertia [kg·m²]
    double r_eff_;    // effective wheel radius [m]
    double lpf_tau_;  // LPF time constant [s]

    // DFO state
    WheelOmegas prev_omegas_{WheelOmegas::Zero()};
    WheelOmegas current_omegas_{WheelOmegas::Zero()};
    WheelTorques motor_torques_{WheelTorques::Zero()};
    Eigen::Vector4d fx_filtered_{Eigen::Vector4d::Zero()};
    double dt_{0.0};
};

}  // namespace tv
