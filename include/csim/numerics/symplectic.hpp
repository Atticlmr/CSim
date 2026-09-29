#pragma once

#include <csim/numerics/integration_detail.hpp>
#include <utility>

namespace csim::numerics {
// Narrow contract: q'=v, v'=acceleration(q). Return {q_next,v_next}.
// For symplectic guarantees require constant positive-definite mass, a force
// from a time-independent potential, and a fixed step. No velocity-dependent
// drag, holonomic constraints or quaternion dynamics in these generic APIs.
namespace detail {
template <typename Vector>
void validateMechanical(const Vector& q,const Vector& v,double dt) {
    if (!std::isfinite(dt) || dt<=0 || !stateIsFinite(q) || !stateIsFinite(v))
        throw std::invalid_argument("Mechanical integration requires finite state and positive finite dt");
}
}

template <typename Vector, typename Acceleration>
std::pair<Vector,Vector> symplecticEulerStep(const Vector& position,
                                            const Vector& velocity,double dt,
                                            Acceleration&& acceleration) {
    detail::validateMechanical(position,velocity,dt);
    const Vector a=acceleration(position); detail::checkComputedState(a);
    const Vector v=velocity+a*dt; detail::checkComputedState(v);
    const Vector q=position+v*dt; detail::checkComputedState(q);
    return {q,v};
}

template <typename Vector, typename Acceleration>
std::pair<Vector,Vector> velocityVerletStep(const Vector& position,
                                          const Vector& velocity,double dt,
                                          Acceleration&& acceleration) {
    detail::validateMechanical(position,velocity,dt);
    const Vector a=acceleration(position); detail::checkComputedState(a);
    const Vector half_velocity=velocity+a*(dt*0.5); detail::checkComputedState(half_velocity);
    const Vector q=position+half_velocity*dt; detail::checkComputedState(q);
    const Vector next_a=acceleration(q); detail::checkComputedState(next_a);
    const Vector v=half_velocity+next_a*(dt*0.5); detail::checkComputedState(v);
    return {q,v};
}
} // namespace csim::numerics
