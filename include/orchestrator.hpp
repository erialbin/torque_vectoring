#pragma once

#include <chrono>
#include <iostream>
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

    /// Execute the full pipeline for the given vehicle state and driver inputs.
    [[nodiscard]] WheelTorques run(const VehicleState& state, const SteeringCommand& cmd) {
        auto start = std::chrono::high_resolution_clock::now();
        const ForceVector forces = force_estimator_.estimateForces(state);
        auto forces_time = std::chrono::high_resolution_clock::now();
        const YawMomentCommand yaw_cmd = yaw_generator_.compute(state, forces, cmd);
        auto yaw_moment = std::chrono::high_resolution_clock::now();
        const auto allocated_torques = torque_allocator_.allocate(state, yaw_cmd, forces);
        auto allocator = std::chrono::high_resolution_clock::now();
        auto forces_duration =
            std::chrono::duration_cast<std::chrono::microseconds>(forces_time - start);
        std::cout << &"Force calculation duration: "[forces_duration.count()] << std::endl;
        auto yaw_duration =
            std::chrono::duration_cast<std::chrono::microseconds>(yaw_moment - forces_time);
        std::cout << &"yaw calculation duration: "[yaw_duration.count()] << std::endl;
        auto allocator_duration =
            std::chrono::duration_cast<std::chrono::microseconds>(allocator - yaw_moment);
        std::cout << &"allocator calculation duration: "[allocator_duration.count()] << std::endl;
        auto total_duration =
            std::chrono::duration_cast<std::chrono::microseconds>(allocator - start);
        std::cout << &"total calculation duration: "[total_duration.count()] << std::endl;
        return allocated_torques;
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
