#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

class KinematicForceEstimator : IForceEstimator<KinematicForceEstimator> {
   public:
    ForceVector estimateForcesImpl(const VehicleState& s) {
        // [Fx_FL, Fx_FR, Fx_RL, Fx_RR, Fy_FL, Fy_FR, Fy_RL, Fy_RR, Fz_FL, Fz_FR, Fz_RL, Fz_RR]
        ForceVector forces = ForceVector::zeros();
        forces[8] =
    };

   private:
    double mass_;
    double cg_height_;
    double wheelbase_;
    double trackwidth_;
    double front_rear_roll_stiffness_;
};

}  // namespace tv
