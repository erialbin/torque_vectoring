#pragma once

#include <concepts>

#include "types.hpp"

namespace tv {

/// Type must provide: ForceVector estimate(const VehicleState&)
template <typename T>
concept ForceEstimator = requires(T estimator, const VehicleState& state) {
    { estimator.estimateForcesImpl(state) } -> std::same_as<ForceVector>;
};

/// Type must provide: YawMoment compute(const VehicleState&, const ForceVector&)
template <typename T>
concept YawMomentGenerator =
    requires(T generator, const VehicleState& state, const ForceVector& forces) {
        { generator.computeImpl(state, forces) } -> std::same_as<YawMoment>;
    };

/// Type must provide: WheelTorques allocate(const VehicleState&, YawMoment)
template <typename T>
concept TorqueAllocator = requires(T allocator, const VehicleState& state, YawMoment moment) {
    { allocator.allocate(state, moment) } -> std::same_as<WheelTorques>;
};

template <YawMomentGenerator Derived>
class IYawController {
   public:
    YawMoment compute(const VehicleState& s) { return static_cast<Derived*>(this)->computeImpl(s); }
};

template <ForceEstimator Derived>
class IForceEstimator {
   public:
    ForceVector estimateForces(const VehicleState& s) {
        return static_cast<Derived*>(this)->estimateForcesImpl(s);
    }
};

template <TorqueAllocator Derived>
class ITorqueAllocator {
   public:
    TorqueVector allocateTorque(const YawMoment& yaw) {
        return static_cast<Derived*>(this)->allocateTorqueImpl(yaw);
    }
}

}  // namespace tv
