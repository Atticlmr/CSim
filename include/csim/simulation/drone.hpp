#pragma once

#include <csim/dynamics/drone.hpp>
#include <csim/simulation/integration.hpp>
#include <csim/model/rigid_body.hpp>

#include <memory>
#include <utility>

namespace csim::simulation {

class DroneModel {
public:
    DroneModel(double mass = 1, math::Matrix3 inertia_B = dynamics::Drone::defaultInertia(),
               double gravity = 9.80665, double timestep = 0.001,
               std::shared_ptr<const model::RigidBodyAsset> asset = {}, dynamics::DragConfig drone_drag = {}, dynamics::WindField wind = {}, IntegratorSettings integration = {})
        : physics_(mass, inertia_B, gravity, drone_drag, wind), timestep_(timestep), integration_(std::move(integration)), asset_(std::move(asset)) {
        integration_.validate(false);
        if (!std::isfinite(timestep_) || timestep_ <= 0) {
            throw std::invalid_argument("Simulation timestep must be finite and positive");
        }
    }
    const dynamics::Drone& physics() const noexcept { return physics_; }
    double timestep() const noexcept { return timestep_; }
    const IntegratorSettings& integration() const noexcept { return integration_; }
    const auto& asset() const noexcept { return asset_; }
private:
    const dynamics::Drone physics_;
    const double timestep_;
    const IntegratorSettings integration_;
    const std::shared_ptr<const model::RigidBodyAsset> asset_;
};

struct DroneSnapshot {
    double time;
    dynamics::DroneState state;
    dynamics::DroneControl control;
    dynamics::DroneObservables physical;
};

class DroneData;
DroneData makeData(std::shared_ptr<DroneModel> model, dynamics::DroneState state = {});
void step(const DroneModel& model, DroneData& data);
DroneSnapshot getState(const DroneModel& model, const DroneData& data);
void setControl(const DroneModel& model, DroneData& data, dynamics::DroneControl control);
void reset(const DroneModel& model, DroneData& data, dynamics::DroneState state = {});

class DroneData {
public:
    std::shared_ptr<DroneModel> model() const { return model_; }
private:
    DroneData(std::shared_ptr<DroneModel> model, dynamics::DroneState state)
        : model_(std::move(model)), state_(state) {}
    friend DroneData makeData(std::shared_ptr<DroneModel>, dynamics::DroneState);
    friend void step(const DroneModel&, DroneData&);
    friend DroneSnapshot getState(const DroneModel&, const DroneData&);
    friend void setControl(const DroneModel&, DroneData&, dynamics::DroneControl);
    friend void reset(const DroneModel&, DroneData&, dynamics::DroneState);
    friend class DroneBatch;
    std::shared_ptr<DroneModel> model_;
    dynamics::DroneState state_;
    dynamics::DroneControl control_{};
    double time_ = 0;
};

inline DroneData makeData(std::shared_ptr<DroneModel> model, dynamics::DroneState state) {
    if (!model) throw std::invalid_argument("Simulation model must not be null");
    const auto candidate = dynamics::Drone::normalizedState(state);
    (void)model->physics().observe(candidate, {});
    return DroneData(std::move(model), candidate);
}

inline DroneSnapshot getState(const DroneModel& model, const DroneData& data) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    return {data.time_, data.state_, data.control_, model.physics().observe(data.state_, data.control_, data.time_)};
}

// Actual physical wrench, held constant during the next physical timestep.
inline void setControl(const DroneModel& model, DroneData& data, dynamics::DroneControl control) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    (void)model.physics().observe(data.state_, control, data.time_);
    data.control_ = control;
}

inline void step(const DroneModel& model, DroneData& data) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    const auto candidate = integration_detail::advance(model.integration(),data.time_,data.state_,model.timestep(),
        [&](double t,const dynamics::DroneState& state,int) { return model.physics().derivative(state,data.control_,t); },
        [](dynamics::DroneState state) { return dynamics::Drone::normalizedState(state); },
        [&](const dynamics::DroneState& state,double t) { (void)model.physics().observe(state,data.control_,t); });
    data.state_ = candidate;
    data.time_ += model.timestep();
}

inline void reset(const DroneModel& model, DroneData& data, dynamics::DroneState state) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    const auto candidate = dynamics::Drone::normalizedState(state);
    (void)model.physics().observe(candidate, {});
    data.state_ = candidate;
    data.control_ = {}; // Reset returns to zero input, not hover thrust.
    data.time_ = 0;
}

} // namespace csim::simulation
