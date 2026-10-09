#pragma once

#include <csim/simulation/integration.hpp>
#include <csim/simulation/constrained.hpp>
#include <vector>

namespace csim::simulation {
struct CableEventSettings {
    double max_step = 0.005;       // Maximum interval between guard samples (s).
    double time_tolerance = 1e-10; // Root bracket width (s), not ODE accuracy.
    std::size_t max_events = 64;
    void validate() const {
        if (!std::isfinite(max_step) || max_step <= 0 || !std::isfinite(time_tolerance)
            || time_tolerance <= 0 || time_tolerance > max_step || !max_events || max_events > 1000000)
            throw std::invalid_argument("Invalid cable event settings");
    }
};

struct CableEvent {
    std::string type; // release or impact (includes zero-impulse engagement).
    double time = 0;
    math::Vector3 impulse_W{}; // On drone; opposite on payload.
    double energy_loss = 0;    // Kinetic loss across the velocity reset only.
    double radial_velocity_before = 0;
    double radial_velocity_after = 0;
    bool slack_after = false;
};

namespace cable_detail {
inline void prependControlRelease(std::vector<CableEvent>& events, double time,
                                  std::size_t max_events) {
    if (events.size()>=max_events)
        throw numerics::IntegrationFailure("Cable event limit exceeded");
    events.insert(events.begin(),CableEvent{"release",time,{},0,0,0,true});
}

using State = dynamics::SuspendedPayloadState;
inline State release(State state, double length) {
    state.payload_position_W=state.drone.position_W+state.cable_direction_W*length;
    state.payload_velocity_W=state.drone.velocity_W
        +state.cable_angular_velocity_W.cross(state.cable_direction_W)*length;
    state.slack=true;
    return state;
}
// No impulse at release. Also used at public input boundaries to avoid reporting
// an unphysical compressive cable force while waiting for the next step.
inline State admissible(const dynamics::SuspendedPayload& physics, State state,
                        const dynamics::DroneControl& control, double time) {
    if (!state.slack && physics.tautTension(state,control,time)<=0)
        state=release(state,physics.length());
    return state;
}

struct Result {
    State state;
    std::vector<CableEvent> events;
    math::Vector3 impulse_W{};
};

inline Result advance(const dynamics::SuspendedPayload& physics, const IntegratorSettings& integration,
                      const CableEventSettings& settings, State initial,
                      const dynamics::DroneControl& control, double start, double dt) {
    numerics::detail::validateExplicitTime(start,dt);
    Result result{initial,{}, {}};
    const double end=start+dt, length=physics.length();
    const double mq=physics.drone().mass(), ml=physics.payloadMass();
    const double mu=mq*(ml/(mq+ml));
    const double geometry_epsilon=64*std::numeric_limits<double>::epsilon()*length;
    std::size_t work=0;
    auto propagate=[&](const State& state,double t,double h) {
        if (integration.method=="rattle") {
            if (++work>integration.max_substeps) throw numerics::IntegrationFailure("Cable event search exceeded max_substeps");
            return integration_detail::pointRattle(physics,state,control,t,h,false);
        }
        return integration_detail::advance(integration,t,state,h,
            [&](double stage,const State& s,int) { return physics.derivative(s,control,stage,false); },
            [&](State s) { return dynamics::SuspendedPayload::projectedState(s,length); },
            [&](const State& s,double) {
                if (++work>integration.max_substeps)
                    throw numerics::IntegrationFailure("Cable event search exceeded max_substeps");
                numerics::detail::checkComputedState(s);
            });
    };
    auto guard=[&](const State& s,double t) {
        const double value=s.slack ? (s.payload_position_W-s.drone.position_W).norm()-length
                                   : -physics.tautTension(s,control,t);
        if (!std::isfinite(value)) throw std::overflow_error("Non-finite cable event guard");
        return value;
    };
    auto append=[&](CableEvent event) {
        if (result.events.size()>=settings.max_events)
            throw numerics::IntegrationFailure("Cable event limit exceeded");
        result.events.push_back(std::move(event));
    };
    auto impact=[&](State& s,double time) {
        const auto delta=s.payload_position_W-s.drone.position_W;
        const double distance=delta.norm();
        const auto direction=delta/distance;
        const double radial=(s.payload_velocity_W-s.drone.velocity_W).dot(direction);
        if (radial < -64*std::numeric_limits<double>::epsilon()
                *std::max(1.0,(s.payload_velocity_W-s.drone.velocity_W).norm()))
            throw numerics::IntegrationFailure("Cable crossing is not outward; use a higher-order integrator or reduce event_max_step");
        // Only roundoff/root-bracket error is corrected, preserving the CoM.
        const auto correction=direction*(distance-length);
        s.drone.position_W+=correction*(ml/(mq+ml));
        s.payload_position_W-=correction*(mq/(mq+ml));
        const double impulse=mu*std::max(0.0,radial);
        s.drone.velocity_W+=direction*(impulse/mq);
        s.payload_velocity_W-=direction*(impulse/ml);
        s.cable_direction_W=direction;
        s.cable_angular_velocity_W=direction.cross(s.payload_velocity_W-s.drone.velocity_W)/length;
        s.slack=false;
        s=dynamics::SuspendedPayload::projectedState(s,length);
        s=admissible(physics,s,control,time);
        const double loss=0.5*impulse*std::max(0.0,radial);
        if (!std::isfinite(loss) || !s.isFinite()) throw std::overflow_error("Cable impulse overflow");
        append({"impact",time,direction*impulse,loss,radial,0,s.slack});
        result.impulse_W+=direction*impulse;
    };
    double time=start;
    while (time<end) {
        auto& state=result.state;
        if (!state.slack && physics.tautTension(state,control,time)<=0) {
            state=release(state,length);
            append({"release",time,{},0,0,0,true});
        }
        if (state.slack) {
            const auto delta=state.payload_position_W-state.drone.position_W;
            const double distance=delta.norm();
            if (distance>=length-geometry_epsilon && distance>0) {
                const auto direction=delta/distance;
                const auto relative=state.payload_velocity_W-state.drone.velocity_W;
                const double radial=relative.dot(direction);
                const double velocity_epsilon=64*std::numeric_limits<double>::epsilon()*std::max(1.0,relative.norm());
                // At a stationary boundary the unconstrained radial acceleration
                // decides engagement. An inward trajectory always stays slack.
                if (radial>velocity_epsilon || (std::abs(radial)<=velocity_epsilon
                        && physics.tautTension(state,control,time)>0)) {
                    impact(state,time);
                }
            }
        }
        double target=std::min(end,time+settings.max_step);
        if (end-target<=8*std::numeric_limits<double>::epsilon()*std::max(std::abs(end),std::abs(time)))
            target=end;
        if (!(target>time)) throw numerics::IntegrationFailure("Cable event time cannot advance");
        auto candidate=propagate(state,time,target-time);
        const double before=guard(state,time), after=guard(candidate,target);
        // Neutral free fall at d=l is not an impact; only an outward crossing
        // or a positive-to-nonpositive tension transition triggers a root.
        const bool crossing=state.slack ? (after>geometry_epsilon || (before<-geometry_epsilon && after>=0))
                                         : after>=0;
        if (!crossing) { state=candidate; time=target; continue; }
        double lo=time, hi=target;
        // Each trial restarts from the same pre-event state and input, extending
        // that smooth branch through the bracket. Switch modes only at the
        // located root; no rejected trial mutates public data.
        while (hi-lo>settings.time_tolerance) {
            const double middle=lo+(hi-lo)*0.5;
            if (!(middle>lo && middle<hi))
                throw numerics::IntegrationFailure("Cable event root cannot be resolved in time");
            auto trial=propagate(state,time,middle-time);
            if (guard(trial,middle)>=0) { hi=middle; candidate=trial; }
            else lo=middle;
        }
        time=hi;
        state=candidate;
        if (state.slack) impact(state,time);
        else {
            state=release(state,length);
            append({"release",time,{},0,0,0,true});
        }
    }
    (void)physics.observe(result.state,control,end,false);
    if (!result.impulse_W.isFinite()) throw std::overflow_error("Cable impulse sum overflow");
    return result;
}
} // namespace cable_detail
} // namespace csim::simulation
