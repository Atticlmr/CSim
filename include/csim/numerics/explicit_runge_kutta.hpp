#pragma once

#include <csim/numerics/integration_detail.hpp>

namespace csim::numerics {
namespace detail {
inline void validateExplicitTime(double t, double dt, bool midpoint=false) {
    if (!std::isfinite(t) || !std::isfinite(dt) || dt<=0)
        throw std::invalid_argument("Explicit integration requires finite time and positive finite dt");
    const double end=t+dt, middle=t+dt*0.5;
    if (!std::isfinite(end) || !(end>t) || (midpoint && !(t<middle && middle<end)))
        throw std::overflow_error("Integration stage times are not distinct finite values");
}
}

// Same value-state and lvalue dynamics contract as rk4Step. No projection.
template <typename State, typename Control, typename Dynamics>
State eulerStep(double t, const State& state, const Control& control,
                double dt, Dynamics&& dynamics) {
    detail::validateExplicitTime(t,dt);
    if (!detail::stateIsFinite(state)) throw std::invalid_argument("Initial state must be finite");
    const State rate=dynamics(t,state,control); detail::checkComputedState(rate);
    const State result=state+rate*dt; detail::checkComputedState(result);
    return result;
}

// Explicit midpoint RK2. This is not the implicit, symplectic midpoint method.
template <typename State, typename Control, typename Dynamics>
State midpointStep(double t, const State& state, const Control& control,
                   double dt, Dynamics&& dynamics) {
    detail::validateExplicitTime(t,dt,true);
    if (!detail::stateIsFinite(state)) throw std::invalid_argument("Initial state must be finite");
    const State first=dynamics(t,state,control); detail::checkComputedState(first);
    const State middle=state+first*(dt*0.5); detail::checkComputedState(middle);
    const State rate=dynamics(t+dt*0.5,middle,control); detail::checkComputedState(rate);
    const State result=state+rate*dt; detail::checkComputedState(result);
    return result;
}

// Explicit trapezoidal (Heun) RK2: predictor then average endpoint slopes.
template <typename State, typename Control, typename Dynamics>
State heunStep(double t, const State& state, const Control& control,
               double dt, Dynamics&& dynamics) {
    detail::validateExplicitTime(t,dt);
    if (!detail::stateIsFinite(state)) throw std::invalid_argument("Initial state must be finite");
    const State first=dynamics(t,state,control); detail::checkComputedState(first);
    const State predictor=state+first*dt; detail::checkComputedState(predictor);
    const State last=dynamics(t+dt,predictor,control); detail::checkComputedState(last);
    const State result=state+(first*0.5+last*0.5)*dt; detail::checkComputedState(result);
    return result;
}
} // namespace csim::numerics
