#pragma once
#include <csim/numerics/rk4.hpp>
#include <csim/numerics/explicit_runge_kutta.hpp>
#include <csim/math/quaternion.hpp>

namespace csim::numerics {
inline math::Quaternion rotationExp(math::Vector3 rotation) {
    const double angle=rotation.norm();
    if (!std::isfinite(angle)) throw std::overflow_error("Rotation exponential overflow");
    if (angle==0) return {};
    return math::Quaternion::fromAxisAngle(rotation,angle);
}
inline math::Vector3 dexpInverse(math::Vector3 rotation,math::Vector3 rate,bool body_rate) {
    const auto cross=rotation.cross(rate);
    const auto result=rate+cross*(body_rate ? .5 : -.5)+rotation.cross(cross)*(1./12);
    if (!result.isFinite()) throw std::overflow_error("Lie algebra derivative overflow");
    return result;
}
template<class State,class Geometry> class LieGroupIntegrator {
public:
    template<class Control,class Dynamics>
    static State step(double time,const State& state,const Control& control,double dt,
                      Dynamics&& dynamics,bool fourth_order=true) {
        const auto initial=Geometry::chart(state);
        auto evaluate=[&](double stage,const State& chart,const Control& input) {
            const auto physical=Geometry::retract(state,chart);
            return Geometry::rate(chart,physical,dynamics(stage,physical,input));
        };
        const auto result=fourth_order ? rk4Step(time,initial,control,dt,evaluate)
                                      : midpointStep(time,initial,control,dt,evaluate);
        return Geometry::retract(state,result);
    }
};
}
