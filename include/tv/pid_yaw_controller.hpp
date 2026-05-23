#pragma once

#include <cmath>

#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

// High-level yaw moment controller — docs/tv.md §layer 2
//
//   Feedforward (yaw moment): bicycle model steady-state yaw moment
//   Feedback    (yaw moment): super-twisting Sliding Mode Control (Goggia et al. 2015)
//   Fx demand               : PI velocity tracker (TU Munich / KIT FSG approach)
//
// SteeringCommand layout: [delta (rad), vx_ref (m/s), r_ref (rad/s)]
// Call setDt(dt) once per control cycle before computeImpl.
class PIDYawController : public IYawController<PIDYawController> {
   public:
    // Bicycle model params (yaw moment feedforward):
    //   c_f           front axle cornering stiffness [N/rad]
    //   c_r           rear axle cornering stiffness [N/rad]
    //   l_f           front axle to CG [m]
    //   l_r           rear axle to CG [m]
    //
    // Loss feedforward params:
    //   rho           air density [kg/m³]
    //   c_d           drag coefficient
    //   frontal_area  frontal reference area [m²]
    //   c_rr          rolling resistance coefficient
    //
    // Longitudinal PI params:
    //   mass          vehicle mass [kg]
    //   k_p           proportional gain [1/s]
    //   k_i           integral gain     [1/s²]
    //   eta           drivetrain efficiency (0, 1]
    //
    //   Fx_total = (k_p·ev + k_i·∫ev dt) · mass / eta + f_loss
    //   f_loss = f_aero + f_roll  (only when accelerating, zero when braking)
    PIDYawController(double c_f, double c_r, double l_f, double l_r, double rho, double c_d,
                     double frontal_area, double c_rr, double kp_yaw, double ki_yaw, double mass,
                     double k_p, double k_i, double eta)
        : c_f_(c_f),
          c_r_(c_r),
          l_f_(l_f),
          l_r_(l_r),
          rho_(rho),
          c_d_(c_d),
          frontal_area_(frontal_area),
          c_rr_(c_rr),
          kp_yaw_(kp_yaw),
          ki_yaw_(ki_yaw),
          mass_(mass),
          kp_longi_(k_p),
          ki_longi_(k_i),
          eta_(eta) {}

    // Must be called once per control cycle before computeImpl.
    void setDt(double dt) { dt_ = dt; }

    // VehicleState:    [vx, vy, yaw_rate, steering_angle, ax, ay]
    // SteeringCommand: uses steering_angle_, vx_ref_, r_ref_
    [[nodiscard]] YawMomentCommand computeImpl(const VehicleState& s, const ForceVector& forces,
                                               const SteeringCommand& cmd) {
        (void)forces;  // reserved for future combined-slip feedforward

        const double vx = s.vx_;
        const double vy = s.vy_;
        const double r = s.yaw_rate_;
        const double delta = cmd.steering_angle_;
        const double vx_ref = cmd.vx_ref_;
        const double r_ref = cmd.r_ref_;

        // -----------------------------------------------------------------------
        // Yaw moment feedforward — bicycle model steady-state
        // -----------------------------------------------------------------------
        double mz_ff = 0.0;
        if (std::abs(vx) > 0.5) {
            const double alpha_f = delta - ((vy + (l_f_ * r_ref)) / vx);
            const double alpha_r = -((vy - (l_r_ * r_ref)) / vx);
            mz_ff = ((l_r_ * c_r_) * alpha_r) - ((l_f_ * c_f_) * alpha_f);
        }

        // PID for yaw rate tracking, no D term right now
        const double e_r = r_ref - r;
        e_r_integral_ += e_r * dt_;  // TODO: maybe need some anti windup

        const double mz_pid = (kp_yaw_ * e_r) + (ki_yaw_ * e_r_integral_);

        const double mz = mz_ff + mz_pid;

        // -----------------------------------------------------------------------
        // Fx demand — PI velocity tracker + loss feedforward (TU Munich / KIT FSG approach)
        //
        //   Fx_total = (k_p·ev + k_i·∫ev dt) · m / η + f_loss
        //
        // Loss feedforward reduces integral windup by compensating known disturbances.
        // Applied only during acceleration — during braking the losses already assist
        // deceleration so adding them would oppose the braking demand.
        // -----------------------------------------------------------------------
        const double e_vx = vx_ref - vx;
        e_vx_integral_ += e_vx * dt_;
        const double f_aero = (0.5 * rho_ * c_d_ * frontal_area_) * (vx * vx);
        const double f_roll = c_rr_ * mass_ * 9.81;
        const double f_loss = (e_vx > 0.0) ? (f_aero + f_roll) : 0.0;
        const double fx_total =
            (((kp_longi_ * e_vx) + (ki_longi_ * e_vx_integral_)) * (mass_ / eta_)) +
            f_loss;  // limiting currently happens in torque allocator

        return YawMomentCommand{.mz_ = mz, .fx_total_ = fx_total};
    }

   private:
    // Bicycle model parameters (yaw moment feedforward)
    const double c_f_;  // front cornering stiffness [N/rad]
    const double c_r_;  // rear cornering stiffness [N/rad]
    const double l_f_;  // front axle to CG [m]
    const double l_r_;  // rear axle to CG [m]

    // Loss feedforward parameters
    const double rho_;           // air density [kg/m³]
    const double c_d_;           // drag coefficient
    const double frontal_area_;  // frontal reference area [m²]
    const double c_rr_;          // rolling resistance coefficient

    // PID Yaw parameters
    const double kp_yaw_;
    const double ki_yaw_;

    // Longitudinal PI parameters
    const double mass_;      // vehicle mass [kg]
    const double kp_longi_;  // proportional gain [1/s]
    const double ki_longi_;  // integral gain     [1/s²]
    const double eta_;       // drivetrain efficiency

    // Yaw PID state
    double e_r_integral_{0.0};  // integral of yaw rate error [rad]

    // PI state
    double e_vx_integral_{0.0};  // integral of velocity error [m]

    // Time step — updated each cycle via setDt()
    double dt_{0.0};
};

}  // namespace tv
