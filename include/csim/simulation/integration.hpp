#pragma once

#include <csim/numerics/rk4.hpp>
#include <csim/numerics/explicit_runge_kutta.hpp>
#include <csim/numerics/dormand_prince.hpp>
#include <csim/numerics/dop853.hpp>
#include <csim/numerics/symplectic.hpp>
#include <csim/numerics/implicit.hpp>
#include <csim/simulation/state_codec.hpp>
#include <csim/simulation/lie_geometry.hpp>
#include <csim/dynamics/pendulum.hpp>
#include <csim/dynamics/suspended_payload.hpp>
#include <csim/dynamics/rigid_payload.hpp>
#include <string>

namespace csim::simulation {
// Model configuration only. No Python callbacks or integrator objects cross the API.
struct IntegratorSettings {
    std::string method="rk4";
    double rtol=1e-6, atol=1e-9;
    std::size_t max_substeps=10000;
    void validate(bool mechanical=false,bool lie=true,bool rattle=false) const {
        const bool general=method=="rk4" || method=="euler" || method=="midpoint"
            || method=="heun" || method=="dopri5" || method=="implicit_euler" || method=="implicit_midpoint"
            || method=="radau5" || method=="bdf1" || method=="bdf2" || method=="dop853";
        if (!general && !(mechanical && (method=="symplectic_euler" || method=="velocity_verlet"))
            && !(lie && (method=="lie_midpoint" || method=="lie_rk4")) && !(rattle && method=="rattle"))
            throw std::invalid_argument("Unsupported integrator for this model");
        if (!std::isfinite(rtol) || rtol<0 || !std::isfinite(atol) || atol<=0
            || !max_substeps || max_substeps>1000000)
            throw std::invalid_argument("Require finite rtol >= 0, atol > 0 and max_substeps in 1..1000000");
    }
};

namespace integration_detail {
// Maximum normalized embedded error across raw SI state components. Quaternion
// components are dimensionless; simultaneous q -> -q leaves this metric unchanged.
// Raw error includes radial drift before any projection; this is not a geodesic norm.
class ErrorNorm {
public:
    explicit ErrorNorm(const IntegratorSettings& settings):scalar_(settings.rtol,settings.atol) {}
    double operator()(double a,double b,double e) const { return scalar_(a,b,e); }
    double operator()(const math::Vector3& a,const math::Vector3& b,const math::Vector3& e) const {
        return std::max({scalar_(a.x,b.x,e.x),scalar_(a.y,b.y,e.y),scalar_(a.z,b.z,e.z)});
    }
    double operator()(const math::Quaternion& a,const math::Quaternion& b,const math::Quaternion& e) const {
        return std::max({scalar_(a.w,b.w,e.w),scalar_(a.x,b.x,e.x),scalar_(a.y,b.y,e.y),scalar_(a.z,b.z,e.z)});
    }
    double operator()(const dynamics::PendulumState& a,const dynamics::PendulumState& b,const dynamics::PendulumState& e) const {
        return std::max(scalar_(a.angle,b.angle,e.angle),scalar_(a.angular_velocity,b.angular_velocity,e.angular_velocity));
    }
    double operator()(const dynamics::DroneState& a,const dynamics::DroneState& b,const dynamics::DroneState& e) const {
        return std::max({(*this)(a.position_W,b.position_W,e.position_W),(*this)(a.velocity_W,b.velocity_W,e.velocity_W),
            (*this)(a.q_WB,b.q_WB,e.q_WB),(*this)(a.angular_velocity_B,b.angular_velocity_B,e.angular_velocity_B)});
    }
    double operator()(const dynamics::SuspendedPayloadState& a,const dynamics::SuspendedPayloadState& b,const dynamics::SuspendedPayloadState& e) const {
        if (a.slack) return std::max({(*this)(a.drone,b.drone,e.drone),
            (*this)(a.payload_position_W,b.payload_position_W,e.payload_position_W),
            (*this)(a.payload_velocity_W,b.payload_velocity_W,e.payload_velocity_W)});
        return std::max({(*this)(a.drone,b.drone,e.drone),
            (*this)(a.cable_direction_W,b.cable_direction_W,e.cable_direction_W),
            (*this)(a.cable_angular_velocity_W,b.cable_angular_velocity_W,e.cable_angular_velocity_W)});
    }
    double operator()(const dynamics::RigidPayloadState& a,const dynamics::RigidPayloadState& b,const dynamics::RigidPayloadState& e) const {
        return std::max((*this)(a.drone,b.drone,e.drone),(*this)(a.payload,b.payload,e.payload));
    }
private:
    numerics::ScalarErrorNorm scalar_;
};

// All accepted internal steps stay local until the outer physical step succeeds.
// Input is held for the entire outer interval. Each accepted substep is projected
// and physically checked; domain exceptions propagate, never masquerade as errors
// to reject/retry past an unimplemented slack/contact event.
template<class State,class Dynamics,class Project,class Validate>
State advance(const IntegratorSettings& settings,double time,const State& state,double dt,
              Dynamics&& dynamics,Project&& project,Validate&& validate) {
    numerics::detail::validateExplicitTime(time,dt);
    auto checked=[&](State candidate,double t) {
        candidate=project(std::move(candidate)); validate(candidate,t); return candidate;
    };
    if (settings.method=="rk4") return checked(numerics::rk4Step(time,state,0,dt,dynamics),time+dt);
    if (settings.method=="euler") return checked(numerics::eulerStep(time,state,0,dt,dynamics),time+dt);
    if (settings.method=="midpoint") return checked(numerics::midpointStep(time,state,0,dt,dynamics),time+dt);
    if (settings.method=="heun") return checked(numerics::heunStep(time,state,0,dt,dynamics),time+dt);
    if (settings.method=="lie_midpoint" || settings.method=="lie_rk4")
        return checked(numerics::LieGroupIntegrator<State,LieGeometry<State>>::step(time,state,0,dt,dynamics,
            settings.method=="lie_rk4"),time+dt);
    const numerics::NewtonOptions nonlinear{std::max(1e-14,std::min(1e-12,settings.atol*.01)),
                                          std::min(1e-10,settings.rtol*.01),20};
    if (settings.method=="implicit_euler" || settings.method=="implicit_midpoint")
        return checked(numerics::implicitThetaStep(time,state,0,dt,dynamics,settings.method=="implicit_euler" ? 1 : .5,nonlinear),time+dt);
    if (settings.method=="radau5" || settings.method=="bdf1" || settings.method=="bdf2") {
        std::size_t work=0;
        auto trial=[&](double start,const State& initial,double interval) {
            if (++work>settings.max_substeps) throw numerics::IntegrationFailure("Implicit integration exceeded max_substeps");
            if (settings.method=="radau5") return numerics::RadauIIA5<State>(nonlinear).step(start,initial,0,interval,dynamics);
            numerics::Bdf<State> solver(start,initial,settings.method=="bdf1" ? 1 : 2,nonlinear);
            const std::size_t segments=settings.method=="bdf1" ? 1 : 2;
            for (std::size_t segment=0;segment<segments;++segment) solver.step(0,interval/segments,dynamics);
            return solver.state();
        };
        const int order=settings.method=="radau5" ? 5 : settings.method=="bdf1" ? 1 : 2;
        const double divisor=std::pow(2.,order)-1;
        const ErrorNorm norm(settings);
        const double end=time+dt;
        State current=state;
        double start=time,interval=dt;
        while (start<end) {
            interval=std::min(interval,end-start);
            numerics::detail::validateRk4Time(start,interval);
            double error=0;
            State fine=current;
            try {
                const auto coarse=trial(start,current,interval);
                const auto middle=trial(start,current,interval*.5);
                fine=trial(start+interval*.5,middle,interval*.5);
                error=norm(current,fine,(fine+coarse*(-1))*(1/divisor));
            } catch (const numerics::IntegrationFailure&) {
                if (work>=settings.max_substeps) throw;
                interval*=.5; continue;
            }
            if (!std::isfinite(error)) throw numerics::IntegrationFailure("Implicit error estimate overflow");
            if (error<=1) {
                start+=interval; current=checked(fine,start);
                if (start==end) return current;
            }
            interval*=error==0 ? 2 : std::clamp(.8*std::pow(error,-1./(order+1)),.2,2.);
        }
        return current;
    }
    if (settings.method!="dopri5" && settings.method!="dop853") throw std::invalid_argument("Expected a general ODE integrator");
    const double end=time+dt;
    const double interval=end-time;
    numerics::AdaptiveStepOptions options;
    options.min_step=std::max(std::numeric_limits<double>::denorm_min(),std::min(1e-12,interval*1e-12));
    options.max_step=interval;
    const ErrorNorm norm(settings);
    auto integrate=[&](auto solver) {
        for (std::size_t index=0;index<settings.max_substeps;++index) {
            solver.step(0,dynamics,norm,end);
            auto candidate=checked(solver.state(),solver.time());
            if (solver.time()==end) return candidate;
            solver.reset(solver.time(),std::move(candidate),solver.nextStepSize());
        }
        throw numerics::IntegrationFailure("Physical step exceeded max_substeps");
    };
    if (settings.method=="dop853") return integrate(numerics::Dop853<State>(time,state,interval,options));
    return integrate(numerics::DormandPrince54<State>(time,state,interval,options));
}
} // namespace integration_detail
} // namespace csim::simulation
