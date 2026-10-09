#pragma once
#include <csim/simulation/lie_geometry.hpp>
#include <csim/numerics/rattle.hpp>

namespace csim::simulation::integration_detail {
inline dynamics::SuspendedPayloadState pointRattle(const dynamics::SuspendedPayload& physics,
        dynamics::SuspendedPayloadState state,const dynamics::DroneControl& control,double time,double dt,
        bool enforce=true) {
    (void)physics.observe(state,control,time,enforce);
    if (state.slack) {
        const auto result=numerics::LieGroupIntegrator<dynamics::SuspendedPayloadState,LieGeometry<dynamics::SuspendedPayloadState>>::step(
            time,state,control,dt,[&](double stage,const auto& value,const auto& input) {
                return physics.derivative(value,input,stage,false);
            },false);
        return dynamics::SuspendedPayload::projectedState(result,physics.length());
    }
    const auto rotational=numerics::LieGroupIntegrator<dynamics::DroneState,LieGeometry<dynamics::DroneState>>::step(
        time,state.drone,control,dt,[&](double stage,const auto& value,const auto& input) {
            return physics.drone().derivative(value,input,stage);
        },false);
    const double drone_mass=physics.drone().mass(),payload_mass=physics.payloadMass();
    const auto payload_position=state.drone.position_W+state.cable_direction_W*physics.length();
    const auto payload_velocity=state.drone.velocity_W+state.cable_angular_velocity_W.cross(state.cable_direction_W)*physics.length();
    using Vector=math::Matrix<6,1>;
    const Vector position{state.drone.position_W.x,state.drone.position_W.y,state.drone.position_W.z,
                          payload_position.x,payload_position.y,payload_position.z};
    const Vector momentum{drone_mass*state.drone.velocity_W.x,drone_mass*state.drone.velocity_W.y,drone_mass*state.drone.velocity_W.z,
                          payload_mass*payload_velocity.x,payload_mass*payload_velocity.y,payload_mass*payload_velocity.z};
    auto separation=[](const Vector& value) { return math::Vector3{value(3,0)-value(0,0),value(4,0)-value(1,0),value(5,0)-value(2,0)}; };
    auto constraint=[&](const Vector& value) { return math::Matrix<1,1>{.5*(separation(value).squaredNorm()-physics.length()*physics.length())}; };
    auto jacobian=[&](const Vector& value) {
        const auto delta=separation(value);
        return math::Matrix<1,6>{-delta.x,-delta.y,-delta.z,delta.x,delta.y,delta.z};
    };
    auto force=[&](double stage,const Vector&) {
        const auto orientation=stage==time ? state.drone.q_WB : rotational.q_WB;
        const auto thrust=orientation.rotate({0,0,control.thrust});
        const double gravity=physics.drone().gravity();
        return Vector{thrust.x,thrust.y,thrust.z-drone_mass*gravity,0,0,-payload_mass*gravity};
    };
    const auto result=numerics::Rattle<6>(Vector{1/drone_mass,1/drone_mass,1/drone_mass,
                                                1/payload_mass,1/payload_mass,1/payload_mass}).step(time,position,momentum,dt,force,constraint,jacobian);
    state.drone=rotational;
    state.drone.position_W={result.first(0,0),result.first(1,0),result.first(2,0)};
    state.drone.velocity_W={result.second(0,0)/drone_mass,result.second(1,0)/drone_mass,result.second(2,0)/drone_mass};
    const auto delta=separation(result.first);
    const math::Vector3 relative{result.second(3,0)/payload_mass-state.drone.velocity_W.x,
        result.second(4,0)/payload_mass-state.drone.velocity_W.y,result.second(5,0)/payload_mass-state.drone.velocity_W.z};
    state.cable_direction_W=delta.normalized(); state.cable_angular_velocity_W=state.cable_direction_W.cross(relative)/physics.length();
    state=dynamics::SuspendedPayload::projectedState(state,physics.length());
    (void)physics.observe(state,control,time+dt,enforce);
    return state;
}
}
