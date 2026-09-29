#pragma once

#include <csim/simulation/drone.hpp>
#include <csim/simulation/suspended_payload.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace csim::simulation {

// Derived observables only: fixed links have no extra integration state.
// Linear quantities refer to the source link origin unless prefixed com_.
// All velocities/accelerations are expressed in W, including angular ones.
struct LinkSnapshot {
    std::string name;
    std::optional<std::string> parent;
    double time;
    model::Pose pose_WL;
    math::Vector3 velocity_W;
    math::Vector3 angular_velocity_W;
    math::Vector3 acceleration_W;
    math::Vector3 angular_acceleration_W;
    math::Vector3 com_position_W;
    math::Vector3 com_velocity_W;
    math::Vector3 com_acceleration_W;
};

namespace detail {
inline LinkSnapshot observeLink(const model::FixedLink& link, double time,
        const dynamics::DroneState& state, const dynamics::DroneObservables& physical) {
    const auto& q = state.q_WB;
    const auto omega = q.rotate(state.angular_velocity_B);
    // d(R*omega_B)/dt = R*alpha_B because omega cross omega = 0.
    const auto alpha = q.rotate(physical.angular_acceleration_B);
    const auto r = q.rotate(link.body_from_link.position);
    const auto orientation = (q*link.body_from_link.orientation).normalized();
    const auto com_r = r + orientation.rotate(link.center_L);
    const auto velocity = [&](const math::Vector3& offset) {
        return state.velocity_W + omega.cross(offset);
    };
    const auto acceleration = [&](const math::Vector3& offset) {
        return physical.acceleration_W + alpha.cross(offset) + omega.cross(omega.cross(offset));
    };
    return {link.name, link.parent, time, {state.position_W+r, orientation},
            velocity(r), omega, acceleration(r), alpha,
            state.position_W+com_r, velocity(com_r), acceleration(com_r)};
}

inline LinkSnapshot observeLink(const model::FixedLink& link, const DroneSnapshot& s) {
    return observeLink(link, s.time, s.state, s.physical);
}
inline LinkSnapshot observeLink(const model::FixedLink& link, const SuspendedPayloadSnapshot& s) {
    // Use the coupled drone acceleration, including the cable force.
    return observeLink(link, s.time, s.state.drone, s.physical.drone);
}
} // namespace detail

template<class Model> std::vector<std::string> linkNames(const Model& model) {
    std::vector<std::string> names;
    if (model.asset()) {
        names.reserve(model.asset()->links.size());
        for (const auto& link : model.asset()->links) names.push_back(link.name);
    }
    return names;
}

template<class Model, class Data>
LinkSnapshot getLinkState(const Model& model, const Data& data, const std::string& name) {
    const auto snapshot = getState(model, data); // Also checks model/data ownership.
    if (model.asset()) {
        for (const auto& link : model.asset()->links)
            if (link.name == name) return detail::observeLink(link, snapshot);
    }
    throw std::out_of_range("Unknown fixed link: " + name);
}

template<class Model, class Data>
std::vector<LinkSnapshot> getLinkStates(const Model& model, const Data& data) {
    const auto snapshot = getState(model, data); // Evaluate physics once for the entire batch.
    std::vector<LinkSnapshot> result;
    if (model.asset()) {
        result.reserve(model.asset()->links.size());
        for (const auto& link : model.asset()->links)
            result.push_back(detail::observeLink(link, snapshot));
    }
    return result;
}

} // namespace csim::simulation
