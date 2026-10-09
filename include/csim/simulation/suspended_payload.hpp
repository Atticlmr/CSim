#pragma once

#include <csim/dynamics/suspended_payload.hpp>
#include <csim/simulation/cable_events.hpp>
#include <csim/model/rigid_body.hpp>

#include <memory>
#include <utility>

namespace csim::simulation {

class SuspendedPayloadModel {
public:
    SuspendedPayloadModel(double drone_mass = 1, double payload_mass = 0.2, double length = 1,
                          math::Matrix3 inertia_B = dynamics::Drone::defaultInertia(),
                          double gravity = 9.80665, double timestep = 0.001,
               std::shared_ptr<const model::RigidBodyAsset> asset = {}, dynamics::DragConfig drone_drag = {}, dynamics::DragConfig payload_drag = {}, dynamics::WindField wind = {}, IntegratorSettings integration = {}, std::string cable_mode = "taut", CableEventSettings events = {})
        : physics_(drone_mass, payload_mass, length, inertia_B, gravity, drone_drag, payload_drag, wind), timestep_(timestep), integration_(std::move(integration)), cable_mode_(std::move(cable_mode)), events_(events), asset_(std::move(asset)) {
        integration_.validate(false,true,true);
        if (integration_.method=="rattle" && (drone_drag.enabled() || payload_drag.enabled()))
            throw std::invalid_argument("RATTLE point payload currently requires zero aerodynamic drag");
        events_.validate();
        if (cable_mode_ != "taut" && cable_mode_ != "hybrid") throw std::invalid_argument("cable_mode must be taut or hybrid");
        if (!std::isfinite(timestep_) || timestep_ <= 0)
            throw std::invalid_argument("Simulation timestep must be finite and positive");
    }
    const dynamics::SuspendedPayload& physics() const noexcept { return physics_; }
    double timestep() const noexcept { return timestep_; }
    const IntegratorSettings& integration() const noexcept { return integration_; }
    const auto& asset() const noexcept { return asset_; }
    const std::string& cableMode() const noexcept { return cable_mode_; }
    const CableEventSettings& events() const noexcept { return events_; }
    bool hybridCable() const noexcept { return cable_mode_ == "hybrid"; }
private:
    const dynamics::SuspendedPayload physics_;
    const double timestep_;
    const IntegratorSettings integration_;
    const std::string cable_mode_;
    const CableEventSettings events_;
    const std::shared_ptr<const model::RigidBodyAsset> asset_;
};

struct SuspendedPayloadSnapshot {
    double time;
    dynamics::SuspendedPayloadState state;
    dynamics::DroneControl control;
    dynamics::SuspendedPayloadObservables physical;
    std::vector<CableEvent> cable_events;
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
    std::vector<CableEvent> events_;
    bool pending_release_ = false;
    double time_ = 0;
};

inline SuspendedPayloadData makeData(std::shared_ptr<SuspendedPayloadModel> model,
    dynamics::DroneControl control, dynamics::SuspendedPayloadState state) {
    if (!model) throw std::invalid_argument("Simulation model must not be null");
    if (state.slack && !model->hybridCable()) throw std::invalid_argument("Slack state requires cable_mode=hybrid");
    auto candidate = dynamics::SuspendedPayload::normalizedState(state,model->physics().length());
    if (model->hybridCable()) candidate=cable_detail::admissible(model->physics(),candidate,control,0);
    (void)model->physics().observe(candidate, control, 0, !model->hybridCable());
    auto data=SuspendedPayloadData(model, control, candidate);
    (void)model->physics().observe(candidate,data.control_, 0, !model->hybridCable());
    return data;
}

inline SuspendedPayloadSnapshot getState(const SuspendedPayloadModel& model, const SuspendedPayloadData& data) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    auto physical=model.physics().observe(data.state_, data.control_, data.time_, !model.hybridCable());
    physical.cable_impulse_W=data.last_impulse_W_;
    return {data.time_, data.state_, data.control_, physical, data.events_};
}

// Actual physical wrench, held constant during the next physical timestep.
inline void setControl(const SuspendedPayloadModel& model, SuspendedPayloadData& data, dynamics::DroneControl control) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    (void)model.physics().observe(data.state_, control, data.time_, !model.hybridCable());
    auto candidate=data.state_;
    auto events=data.events_;
    bool released=false;
    if (model.hybridCable()) {
        candidate=cable_detail::admissible(model.physics(),candidate,control,data.time_);
        released=!data.state_.slack && candidate.slack;
        if (released)
            events.push_back({"release",data.time_,{},0,0,0,true});
    }
    (void)model.physics().observe(candidate,control,data.time_,!model.hybridCable());
    data.events_=std::move(events);
    data.pending_release_=data.pending_release_ || released;
    data.state_=candidate;
    data.control_ = control;
}

inline void step(const SuspendedPayloadModel& model, SuspendedPayloadData& data) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    if (model.hybridCable()) {
        auto result=cable_detail::advance(model.physics(),model.integration(),model.events(),
            data.state_,data.control_,data.time_,model.timestep());
        if (data.pending_release_)
            cable_detail::prependControlRelease(result.events,data.time_,model.events().max_events);
        data.events_=std::move(result.events);
        data.state_=result.state;
        data.last_impulse_W_=result.impulse_W;
    } else {
        auto candidate=model.integration().method=="rattle"
            ? integration_detail::pointRattle(model.physics(),data.state_,data.control_,data.time_,model.timestep())
            : integration_detail::advance(model.integration(),data.time_,data.state_,model.timestep(),
            [&](double t,const dynamics::SuspendedPayloadState& state,int) { return model.physics().derivative(state,data.control_,t); },
            [&](dynamics::SuspendedPayloadState state) { return dynamics::SuspendedPayload::projectedState(state,model.physics().length()); },
            [&](const dynamics::SuspendedPayloadState& state,double t) { (void)model.physics().observe(state,data.control_,t); });
        data.state_=candidate;
        data.events_.clear();
        data.last_impulse_W_={};
    }
    data.pending_release_=false;
    data.time_+=model.timestep();
}

inline void reset(const SuspendedPayloadModel& model, SuspendedPayloadData& data,
                  dynamics::DroneControl control, dynamics::SuspendedPayloadState state) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    if (state.slack && !model.hybridCable()) throw std::invalid_argument("Slack state requires cable_mode=hybrid");
    auto candidate = dynamics::SuspendedPayload::normalizedState(state,model.physics().length());
    if (model.hybridCable()) candidate=cable_detail::admissible(model.physics(),candidate,control,0);
    (void)model.physics().observe(candidate, control, 0, !model.hybridCable());
    data.state_ = candidate;
    data.control_ = control;
    data.last_impulse_W_={};
    data.events_.clear();
    data.pending_release_=false;
    data.time_ = 0;
}

} // namespace csim::simulation
