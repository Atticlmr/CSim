#pragma once
#include <csim/numerics/lie_group.hpp>
#include <csim/dynamics/rigid_payload.hpp>
#include <csim/dynamics/pendulum.hpp>

namespace csim::simulation::integration_detail {
template<class State> struct LieGeometry {
    static State chart(const State& state) { return state; }
    static State retract(const State&,const State& chart) { return chart; }
    static State rate(const State&,const State&,const State& derivative) { return derivative; }
};
template<> struct LieGeometry<dynamics::DroneState> {
    static dynamics::DroneState chart(dynamics::DroneState state) { state.q_WB={0,0,0,0}; return state; }
    static dynamics::DroneState retract(const dynamics::DroneState& reference,dynamics::DroneState value) {
        value.q_WB=(reference.q_WB*numerics::rotationExp({value.q_WB.x,value.q_WB.y,value.q_WB.z})).normalized();
        return value;
    }
    static dynamics::DroneState rate(const dynamics::DroneState& chart,const dynamics::DroneState& physical,
                                     dynamics::DroneState derivative) {
        const auto rotation=numerics::dexpInverse({chart.q_WB.x,chart.q_WB.y,chart.q_WB.z},physical.angular_velocity_B,true);
        derivative.q_WB={0,rotation.x,rotation.y,rotation.z};
        return derivative;
    }
};
template<> struct LieGeometry<dynamics::RigidPayloadState> {
    static dynamics::RigidPayloadState chart(dynamics::RigidPayloadState state) {
        state.drone=LieGeometry<dynamics::DroneState>::chart(state.drone);
        state.payload=LieGeometry<dynamics::DroneState>::chart(state.payload); return state;
    }
    static dynamics::RigidPayloadState retract(const dynamics::RigidPayloadState& reference,dynamics::RigidPayloadState value) {
        value.drone=LieGeometry<dynamics::DroneState>::retract(reference.drone,value.drone);
        value.payload=LieGeometry<dynamics::DroneState>::retract(reference.payload,value.payload); return value;
    }
    static dynamics::RigidPayloadState rate(const dynamics::RigidPayloadState& chart,const dynamics::RigidPayloadState& physical,
                                            dynamics::RigidPayloadState derivative) {
        derivative.drone=LieGeometry<dynamics::DroneState>::rate(chart.drone,physical.drone,derivative.drone);
        derivative.payload=LieGeometry<dynamics::DroneState>::rate(chart.payload,physical.payload,derivative.payload); return derivative;
    }
};
template<> struct LieGeometry<dynamics::SuspendedPayloadState> {
    static dynamics::SuspendedPayloadState chart(dynamics::SuspendedPayloadState state) {
        state.drone=LieGeometry<dynamics::DroneState>::chart(state.drone);
        if (!state.slack) state.cable_direction_W={}; return state;
    }
    static dynamics::SuspendedPayloadState retract(const dynamics::SuspendedPayloadState& reference,dynamics::SuspendedPayloadState value) {
        value.drone=LieGeometry<dynamics::DroneState>::retract(reference.drone,value.drone);
        if (!value.slack) value.cable_direction_W=numerics::rotationExp(value.cable_direction_W).rotate(reference.cable_direction_W);
        return value;
    }
    static dynamics::SuspendedPayloadState rate(const dynamics::SuspendedPayloadState& chart,
        const dynamics::SuspendedPayloadState& physical,dynamics::SuspendedPayloadState derivative) {
        derivative.drone=LieGeometry<dynamics::DroneState>::rate(chart.drone,physical.drone,derivative.drone);
        if (!physical.slack) derivative.cable_direction_W=numerics::dexpInverse(chart.cable_direction_W,physical.cable_angular_velocity_W,false);
        return derivative;
    }
};
}
