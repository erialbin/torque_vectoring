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
class SmcYawController : public IYawController<SmcYawController> {
   public:
    // Bicycle model params (yaw moment feedforward):
    //   c_f           front axle cornering stiffness [N/rad]
    //   c_r           rear axle cornering stiffness [N/rad]
    //   l_f           front axle to CG [m]
    //   l_r           rear axle to CG [m]
    //
    // SMC (super-twisting) params:
    //   s_lambda      integral weight in sliding surface σ = e_r + s_lambda·∫e_r dt
    //   k1            gain on −k1·√|σ|·sign(σ)
    //   k2            integral gain on −k2·sign(σ)
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
    SmcYawController(double c_f, double c_r, double l_f, double l_r, double rho, double c_d,
                     double frontal_area, double c_rr, double s_lambda, double k1, double k2,
                     double mass, double k_p, double k_i, double eta)
        : c_f_(c_f),
          c_r_(c_r),
          l_f_(l_f),
          l_r_(l_r),
          rho_(rho),
          c_d_(c_d),
          frontal_area_(frontal_area),
          c_rr_(c_rr),
          s_lambda_(s_lambda),
          k1_(k1),
          k2_(k2),
          mass_(mass),
          k_p_(k_p),
          k_i_(k_i),
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
        // Yaw moment feedforward — bicycle model steady-state (docs/tv.md §vehicle dynamics)
        //
        // Steady-state yaw moment balance at r = r_ref:
        //   0 = l_f·C_f·α_f − l_r·C_r·α_r + Mz_ff
        //
        // Tire slip angles evaluated at the reference yaw rate:
        //   α_f = δ − (vy + l_f·r_ref) / vx
        //   α_r = −(vy − l_r·r_ref) / vx
        //
        // Solving: Mz_ff = l_r·C_r·α_r − l_f·C_f·α_f
        // -----------------------------------------------------------------------
        double mz_ff = 0.0;
        if (std::abs(vx) > 0.5) {
            const double alpha_f = delta - ((vy + (l_f_ * r_ref)) / vx);
            const double alpha_r = -((vy - (l_r_ * r_ref)) / vx);
            mz_ff = ((l_r_ * c_r_) * alpha_r) - ((l_f_ * c_f_) * alpha_f);
        }

        // -----------------------------------------------------------------------
        // SMC feedback — super-twisting algorithm (docs/tv.md §layer 2)
        //
        //   Sliding surface:  σ = e_r + s_lambda·∫e_r dt
        //   Control law:      u = −k1·√|σ|·sign(σ) + v
        //                     v̇ = −k2·sign(σ)
        //
        // Reference: Goggia, Sorniotti & Ferrara (IEEE TVT, 2015)
        // -----------------------------------------------------------------------
        const double e_r = r_ref - r;
        e_r_integral_ += e_r * dt_;
        const double sigma = e_r + (s_lambda_ * e_r_integral_);

        double sgn_sigma = 0.0;
        if (sigma > 0.0) {
            sgn_sigma = 1.0;
        } else if (sigma < 0.0) {
            sgn_sigma = -1.0;
        }

        const double u_smc = (-k1_ * std::sqrt(std::abs(sigma)) * sgn_sigma) + v_st_;
        v_st_ += (-k2_ * sgn_sigma) * dt_;

        const double mz = mz_ff + u_smc;

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
        const double fx_total = (((k_p_ * e_vx) + (k_i_ * e_vx_integral_)) * (mass_ / eta_)) + f_loss;

        return YawMomentCommand{.mz_ = mz, .fx_total_ = fx_total};
    }

   private:
    // Bicycle model parameters (yaw moment feedforward)
    double c_f_;  // front cornering stiffness [N/rad]
    double c_r_;  // rear cornering stiffness [N/rad]
    double l_f_;  // front axle to CG [m]
    double l_r_;  // rear axle to CG [m]

    // Loss feedforward parameters
    double rho_;           // air density [kg/m³]
    double c_d_;           // drag coefficient
    double frontal_area_;  // frontal reference area [m²]
    double c_rr_;          // rolling resistance coefficient

    // SMC parameters
    double s_lambda_;  // sliding surface integral weight
    double k1_;        // super-twisting gain on sqrt(|sigma|)
    double k2_;        // super-twisting integral gain on sign(sigma)

    // Longitudinal PI parameters
    double mass_;  // vehicle mass [kg]
    double k_p_;   // proportional gain [1/s]
    double k_i_;   // integral gain     [1/s²]
    double eta_;   // drivetrain efficiency

    // SMC state
    double e_r_integral_{0.0};  // integral of yaw rate error [rad]
    double v_st_{0.0};          // super-twisting integral term [Nm]

    // PI state
    double e_vx_integral_{0.0};  // integral of velocity error [m]

    // Time step — updated each cycle via setDt()
    double dt_{0.0};
};

}  // namespace tv
