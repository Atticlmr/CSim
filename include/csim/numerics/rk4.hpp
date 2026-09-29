#pragma once

#include <cmath>
#include <stdexcept>
#include <csim/numerics/integration_detail.hpp>

namespace csim::numerics {

namespace detail {

inline void validateRk4Time(double t, double dt) {
    if (!std::isfinite(t) || !std::isfinite(dt) || dt <= 0.0) {
        throw std::invalid_argument("RK4 requires finite time and finite positive dt");
    }
    const double midpoint = t + dt * 0.5;
    const double endpoint = t + dt;
    if (!std::isfinite(endpoint) || !(t < midpoint && midpoint < endpoint)) {
        throw std::overflow_error("RK4 stage times are not distinct finite values");
    }
}

} // namespace detail

// Classical fixed-step RK4. See docs/rk4.md for requirements and limitations.
// Dynamics: dynamics(t, state, control) -> State derivative; control is held
// constant across stages. State supports +, right scalar *, and isFinite()
// (floating-point scalar states use std::isfinite). Inputs are never assigned.
// The callable is invoked as an lvalue, without copying or moving between stages.
// No normalization/projection, adaptive control, events or controller updates.
template <typename State, typename Control, typename Dynamics>
State rk4Step(double t, const State& state, const Control& control,
              double dt, Dynamics&& dynamics) {
    detail::validateRk4Time(t, dt);
    if (!detail::stateIsFinite(state)) {
        throw std::invalid_argument("RK4 initial state must be finite");
    }
    const double half_dt = dt * 0.5;
    const double midpoint = t + half_dt;

    const State k1 = dynamics(t, state, control);
    detail::checkComputedState(k1);
    const State stage2 = state + k1 * half_dt;
    detail::checkComputedState(stage2);
    const State k2 = dynamics(midpoint, stage2, control);
    detail::checkComputedState(k2);
    const State stage3 = state + k2 * half_dt;
    detail::checkComputedState(stage3);
    const State k3 = dynamics(midpoint, stage3, control);
    detail::checkComputedState(k3);
    const State stage4 = state + k3 * dt;
    detail::checkComputedState(stage4);
    const State k4 = dynamics(t + dt, stage4, control);
    detail::checkComputedState(k4);

    // Weight before summing to avoid the avoidable overflow of k1+2*k2+2*k3+k4.
    // Apply dt last so dt/6 does not underflow for representable subnormal steps.
    const State result = state + (k1 * (1.0 / 6.0) + k2 * (1.0 / 3.0)
                                  + k3 * (1.0 / 3.0) + k4 * (1.0 / 6.0)) * dt;
    detail::checkComputedState(result);
    return result;
}

} // namespace csim::numerics
