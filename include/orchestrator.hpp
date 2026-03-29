#pragma once

#include <utility>

#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

/// Runs the full torque vectoring pipeline:
///   VehicleState -> ForceEstimator -> YawMomentGenerator -> TorqueAllocator -> WheelTorques
template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
class Orchestrator {
   public:
    Orchestrator(F force_estimator, Y yaw_generator, A torque_allocator)
        : force_estimator_(std::move(force_estimator)),
          yaw_generator_(std::move(yaw_generator)),
          torque_allocator_(std::move(torque_allocator)) {}

    /// Execute the full pipeline for the given vehicle state.
    [[nodiscard]] WheelTorques run(const VehicleState& state) {
        const ForceVector forces = force_estimator_.estimateForces(state);
        const YawMoment moment = yaw_generator_.compute(state, forces);
        return torque_allocator_.allocate(state, moment);
    }

    [[nodiscard]] const F& forceEstimator() const noexcept { return force_estimator_; }
    [[nodiscard]] const Y& yawGenerator() const noexcept { return yaw_generator_; }
    [[nodiscard]] const A& torqueAllocator() const noexcept { return torque_allocator_; }

   private:
    F force_estimator_;
    Y yaw_generator_;
    A torque_allocator_;
};

// CTAD: Orchestrator{f, y, a} deduces template args without explicit spelling
template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
Orchestrator(F, Y, A) -> Orchestrator<F, Y, A>;

}  // namespace tv
