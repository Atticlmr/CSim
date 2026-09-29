#pragma once

#include <csim/dynamics/pendulum.hpp>
#include <csim/simulation/integration.hpp>

#include <memory>
#include <utility>

namespace csim::simulation {

// Immutable model description; each Data owns an independent simulation state.
class PendulumModel {
public:
    PendulumModel(double length = 1, double mass = 1, double gravity = 9.80665,
                  double timestep = 0.001, math::Vector3 pivot_W = {}, IntegratorSettings integration = {})
        : physics_(length, mass, gravity, pivot_W), timestep_(timestep), integration_(std::move(integration)) {
        integration_.validate(true);
        if (!std::isfinite(timestep_) || timestep_ <= 0) {
            throw std::invalid_argument("Simulation timestep must be finite and positive");
        }
    }
    const dynamics::Pendulum& physics() const noexcept { return physics_; }
    double timestep() const noexcept { return timestep_; }
    const IntegratorSettings& integration() const noexcept { return integration_; }
private:
    const dynamics::Pendulum physics_;
    const double timestep_;
    const IntegratorSettings integration_;
};

struct PendulumSnapshot {
    double time;
    dynamics::PendulumState state;
    dynamics::PendulumObservables physical;
};

class PendulumData;
PendulumData makeData(std::shared_ptr<PendulumModel> model, double angle = 0,
                      double angular_velocity = 0);
void step(const PendulumModel& model, PendulumData& data);
PendulumSnapshot getState(const PendulumModel& model, const PendulumData& data);
void reset(const PendulumModel& model, PendulumData& data, double angle = 0,
           double angular_velocity = 0);

class PendulumData {
public:
    std::shared_ptr<PendulumModel> model() const { return model_; }
private:
    PendulumData(std::shared_ptr<PendulumModel> model, dynamics::PendulumState state)
        : model_(std::move(model)), state_(state) {}
    friend PendulumData makeData(std::shared_ptr<PendulumModel>, double, double);
    friend void step(const PendulumModel&, PendulumData&);
    friend PendulumSnapshot getState(const PendulumModel&, const PendulumData&);
    friend void reset(const PendulumModel&, PendulumData&, double, double);
    std::shared_ptr<PendulumModel> model_;
    dynamics::PendulumState state_;
    double time_ = 0;
};

inline PendulumData makeData(std::shared_ptr<PendulumModel> model, double angle, double angular_velocity) {
    if (!model) throw std::invalid_argument("Simulation model must not be null");
    const dynamics::PendulumState state{angle, angular_velocity};
    (void)model->physics().observe(state); // Validate state and observable representability.
    return PendulumData(std::move(model), state);
}

inline PendulumSnapshot getState(const PendulumModel& model, const PendulumData& data) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    return {data.time_, data.state_, model.physics().observe(data.state_)};
}

inline void step(const PendulumModel& model, PendulumData& data) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    numerics::detail::validateExplicitTime(data.time_,model.timestep());
    const auto& method=model.integration().method;
    const auto candidate = [&]() {
        if (method=="symplectic_euler" || method=="velocity_verlet") {
            auto acceleration=[&](double angle) { return model.physics().derivative({angle,0}).angular_velocity; };
            const auto result=method=="symplectic_euler"
                ? numerics::symplecticEulerStep(data.state_.angle,data.state_.angular_velocity,model.timestep(),acceleration)
                : numerics::velocityVerletStep(data.state_.angle,data.state_.angular_velocity,model.timestep(),acceleration);
            const dynamics::PendulumState state{result.first,result.second};
            (void)model.physics().observe(state); return state;
        }
        return integration_detail::advance(model.integration(),data.time_,data.state_,model.timestep(),
            [&](double,const dynamics::PendulumState& state,int) { return model.physics().derivative(state); },
            [](dynamics::PendulumState state) { return state; },
            [&](const dynamics::PendulumState& state,double) { (void)model.physics().observe(state); });
    }();
    // Commit the full external interval only after all internal work succeeds.
    data.state_ = candidate;
    data.time_ += model.timestep();
}

inline void reset(const PendulumModel& model, PendulumData& data, double angle, double angular_velocity) {
    if (&model != data.model_.get()) throw std::invalid_argument("Data belongs to a different model");
    const dynamics::PendulumState candidate{angle, angular_velocity};
    (void)model.physics().observe(candidate);
    data.state_ = candidate;
    data.time_ = 0;
}

} // namespace csim::simulation
