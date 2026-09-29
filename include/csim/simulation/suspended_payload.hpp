#pragma once

#include <csim/dynamics/suspended_payload.hpp>
#include <csim/simulation/integration.hpp>
#include <csim/model/rigid_body.hpp>

#include <memory>
#include <utility>

namespace csim::simulation {

class SuspendedPayloadModel {
public:
    SuspendedPayloadModel(double drone_mass = 1, double payload_mass = 0.2, double length = 1,
                          math::Matrix3 inertia_B = dynamics::Drone::defaultInertia(),
                          double gravity = 9.80665, double timestep = 0.001,
               std::shared_ptr<const model::RigidBodyAsset> asset = {}, dynamics::DragConfig drone_drag = {}, dynamics::DragConfig payload_drag = {}, dynamics::WindField wind = {}, IntegratorSettings integration = {}, std::string cable_mode = "taut")
        : physics_(drone_mass, payload_mass, length, inertia_B, gravity, drone_drag, payload_drag, wind), timestep_(timestep), integration_(std::move(integration)), cable_mode_(std::move(cable_mode)), asset_(std::move(asset)) {
        integration_.validate(false);
        if (cable_mode_ != "taut" && cable_mode_ != "hybrid") throw std::invalid_argument("cable_mode must be taut or hybrid");
        if (!std::isfinite(timestep_) || timestep_ <= 0)
            throw std::invalid_argument("Simulation timestep must be finite and positive");
    }
    const dynamics::SuspendedPayload& physics() const noexcept { return physics_; }
    double timestep() const noexcept { return timestep_; }
    const IntegratorSettings& integration() const noexcept { return integration_; }
    const auto& asset() const noexcept { return asset_; }
    const std::string& cableMode() const noexcept { return cable_mode_; }
    bool hybridCable() const noexcept { return cable_mode_ == "hybrid"; }
private:
    const dynamics::SuspendedPayload physics_;
    const double timestep_;
    const IntegratorSettings integration_;
    const std::string cable_mode_;
    const std::shared_ptr<const model::RigidBodyAsset> asset_;
};

struct SuspendedPayloadSnapshot {
    double time;
    dynamics::SuspendedPayloadState state;
    dynamics::DroneControl control;
    dynamics::SuspendedPayloadObservables physical;
};

class SuspendedPayloadData;
// Explicit initial control: zero thrust at rest is outside the strict taut branch.
SuspendedPayloadData makeData(std::shared_ptr<SuspendedPayloadModel> model,
    dynamics::DroneControl control, dynamics::SuspendedPayloadState state = {});
void step(const SuspendedPayloadModel&, SuspendedPayloadData&);
SuspendedPayloadSnapshot getState(const SuspendedPayloadModel&, const SuspendedPayloadData&);
void setControl(const SuspendedPayloadModel&, SuspendedPayloadData&, dynamics::DroneControl);
void reset(const SuspendedPayloadModel&, SuspendedPayloadData&, dynamics::DroneControl,
           dynamics::SuspendedPayloadState state = {});

class SuspendedPayloadData {
public:
    std::shared_ptr<SuspendedPayloadModel> model() const { return model_; }
private:
    SuspendedPayloadData(std::shared_ptr<SuspendedPayloadModel> model,
                         dynamics::DroneControl control, dynamics::SuspendedPayloadState state)
        : model_(std::move(model)), state_(state), control_(control) {}
    friend SuspendedPayloadData makeData(std::shared_ptr<SuspendedPayloadModel>,
                                        dynamics::DroneControl, dynamics::SuspendedPayloadState);
    friend void step(const SuspendedPayloadModel&, SuspendedPayloadData&);
    friend SuspendedPayloadSnapshot getState(const SuspendedPayloadModel&, const SuspendedPayloadData&);
    friend void setControl(const SuspendedPayloadModel&, SuspendedPayloadData&, dynamics::DroneControl);
    friend void reset(const SuspendedPayloadModel&, SuspendedPayloadData&, dynamics::DroneControl,
                      dynamics::SuspendedPayloadState);
    std::shared_ptr<SuspendedPayloadModel> model_;
    dynamics::SuspendedPayloadState state_;
    dynamics::DroneControl control_;
    math::Vector3 last_impulse_W_{};
    double time_ = 0;
};

inline SuspendedPayloadData makeData(std::shared_ptr<SuspendedPayloadModel> model,
    dynamics::DroneControl control, dynamics::SuspendedPayloadState state) {
    if (!model) throw std::invalid_argument("Simulation model must not be null");
    if (state.slack && !model->hybridCable()) throw std::invalid_argument("Slack state requires cable_mode=hybrid");
    const auto candidate = dynamics::SuspendedPayload::normalizedState(state,model->physics().length());
    (void)model->physics().observe(candidate, control, 0, !model->hybridCable());
    auto data=SuspendedPayloadData(model, control, candidate);
    (void)model->physics().observe(candidate,data.control_, 0, !model->hybridCable());
    return data;
}

inline SuspendedPayloadSnapshot getState(const SuspendedPayloadModel& model, const SuspendedPayloadData& data) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    auto physical=model.physics().observe(data.state_, data.control_, data.time_, !model.hybridCable());
    physical.cable_impulse_W=data.last_impulse_W_;
    return {data.time_, data.state_, data.control_, physical};
}

// Actual physical wrench, held constant during the next physical timestep.
inline void setControl(const SuspendedPayloadModel& model, SuspendedPayloadData& data, dynamics::DroneControl control) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    (void)model.physics().observe(data.state_, control, data.time_, !model.hybridCable());
    data.control_ = control;
}

inline void step(const SuspendedPayloadModel& model, SuspendedPayloadData& data) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    auto candidate=integration_detail::advance(model.integration(),data.time_,data.state_,model.timestep(),
        [&](double t,const dynamics::SuspendedPayloadState& state,int) { return model.physics().derivative(state,data.control_,t,!model.hybridCable()); },
        [&](dynamics::SuspendedPayloadState state) { return dynamics::SuspendedPayload::projectedState(state,model.physics().length()); },
        [&](const dynamics::SuspendedPayloadState& state,double t) {
            if (!state.slack) {
                const double tension=model.physics().tautTension(state,data.control_,t);
                if (tension <= 0 && !model.hybridCable()) throw dynamics::CableDomainError("Taut cable has lost tension");
            }
            if (!state.slack) (void)model.physics().observe(state,data.control_,t,!model.hybridCable());
        });
    data.last_impulse_W_={};
    const double length=model.physics().length();
    if (!model.hybridCable()) {
        (void)model.physics().observe(candidate,data.control_,data.time_+model.timestep(),!model.hybridCable());
    } else if (!candidate.slack) {
        if (model.physics().tautTension(candidate,data.control_,data.time_+model.timestep()) <= 0) {
            const auto release=model.physics().observe(candidate,data.control_,data.time_+model.timestep(),false);
            candidate.payload_position_W=release.payload_position_W;
            candidate.payload_velocity_W=release.payload_velocity_W;
            candidate.slack=true;
            candidate=dynamics::SuspendedPayload::projectedState(candidate,length);
        }
        (void)model.physics().observe(candidate,data.control_,data.time_+model.timestep(),!model.hybridCable());
    } else {
        const auto delta=candidate.payload_position_W-candidate.drone.position_W;
        const double distance=delta.norm();
        if (distance >= length && distance > 1e-12) {
            const auto s=delta/distance;
            const double relative=(candidate.payload_velocity_W-candidate.drone.velocity_W).dot(s);
            candidate.payload_position_W=candidate.drone.position_W+s*length;
            // A cable only engages while the endpoints are separating. An
            // inward boundary touch remains slack and can leave the sphere.
            if (relative <= 0) {
                (void)model.physics().observe(candidate,data.control_,data.time_+model.timestep(),false);
                data.state_=candidate;
                data.time_+=model.timestep();
                return;
            }
            const double impulse=model.physics().drone().mass()*model.physics().payloadMass()
                /(model.physics().drone().mass()+model.physics().payloadMass())*relative;
            candidate.payload_velocity_W-=s*(impulse/model.physics().payloadMass());
            candidate.drone.velocity_W+=s*(impulse/model.physics().drone().mass());
            candidate.slack=false; candidate.cable_direction_W=s;
            const auto tangent=candidate.payload_velocity_W-candidate.drone.velocity_W;
            candidate.cable_angular_velocity_W=s.cross(tangent)/length;
            candidate.payload_position_W={}; candidate.payload_velocity_W={};
            data.last_impulse_W_=s*impulse;
        }
        (void)model.physics().observe(candidate,data.control_,data.time_+model.timestep(),!model.hybridCable());
    }
    data.state_=candidate;
    data.time_+=model.timestep();
}

inline void reset(const SuspendedPayloadModel& model, SuspendedPayloadData& data,
                  dynamics::DroneControl control, dynamics::SuspendedPayloadState state) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    if (state.slack && !model.hybridCable()) throw std::invalid_argument("Slack state requires cable_mode=hybrid");
    const auto candidate = dynamics::SuspendedPayload::normalizedState(state,model.physics().length());
    (void)model.physics().observe(candidate, control, 0, !model.hybridCable());
    data.state_ = candidate;
    data.control_ = control;
    data.last_impulse_W_={};
    data.time_ = 0;
}

} // namespace csim::simulation
