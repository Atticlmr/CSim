#pragma once

#include <csim/math/vector.hpp>

#include <cmath>
#include <stdexcept>

namespace csim::dynamics {

struct PendulumState {
    double angle = 0;            // rad, from downward vertical toward world +X
    double angular_velocity = 0; // rad/s, derivative of angle (not a 3D body rate)
    bool isFinite() const {
        return std::isfinite(angle) && std::isfinite(angular_velocity);
    }
};
inline PendulumState operator+(const PendulumState& a, const PendulumState& b) {
    return {a.angle + b.angle, a.angular_velocity + b.angular_velocity};
}
inline PendulumState operator*(const PendulumState& state, double scalar) {
    return {state.angle * scalar, state.angular_velocity * scalar};
}

class PendulumDomainError : public std::domain_error {
public:
    using std::domain_error::domain_error;
};

struct PendulumObservables {
    math::Vector3 position_W;
    math::Vector3 velocity_W;
    math::Vector3 cable_direction_W;
    double tension;
    double energy; // Relative to the lowest point, J.
};

// Passive planar pendulum in world X-Z, with a fixed pivot and taut massless cable.
// First model is deliberately limited to |angle| < pi/2 and E/(m*g*l) < 1.
// In exact conservative dynamics this regime never reaches zero tension.
// No slack transitions, complete rotations, actuation or small-angle approximation.
class Pendulum {
public:
    Pendulum(double length = 1, double mass = 1, double gravity = 9.80665,
             math::Vector3 pivot_W = {})
        : length_(length), mass_(mass), gravity_(gravity), pivot_W_(pivot_W) {
        if (!std::isfinite(length_) || length_ <= 0 || !std::isfinite(mass_) || mass_ <= 0
            || !std::isfinite(gravity_) || gravity_ <= 0 || !pivot_W_.isFinite()) {
            throw std::invalid_argument("Pendulum requires finite positive length/mass/gravity and finite pivot");
        }
        frequency_squared_ = gravity_ / length_;
        frequency_ = std::sqrt(frequency_squared_);
        weight_ = mass_ * gravity_;
        energy_scale_ = weight_ * length_;
        for (double value : {frequency_squared_, frequency_, weight_, energy_scale_}) {
            if (!std::isfinite(value) || value <= 0) {
                throw std::overflow_error("Pendulum parameter scales are not representable");
            }
        }
    }
    double length() const noexcept { return length_; }
    double mass() const noexcept { return mass_; }
    double gravity() const noexcept { return gravity_; }
    const math::Vector3& pivot() const noexcept { return pivot_W_; }

    void validate(const PendulumState& state) const {
        if (!state.isFinite()) throw std::invalid_argument("Pendulum state must be finite");
        constexpr double half_pi = 1.57079632679489661923;
        if (std::abs(state.angle) >= half_pi || !(dimensionlessEnergy(state) < 1)) {
            throw PendulumDomainError("Pendulum supports only |angle| < pi/2 and E/(m*g*l) < 1");
        }
    }

    PendulumState derivative(const PendulumState& state) const {
        validate(state);
        const PendulumState result{state.angular_velocity, -frequency_squared_ * std::sin(state.angle)};
        if (!result.isFinite()) throw std::overflow_error("Pendulum derivative is not finite");
        return result;
    }

    PendulumObservables observe(const PendulumState& state) const {
        validate(state);
        const double s = std::sin(state.angle), c = std::cos(state.angle);
        const math::Vector3 direction{s, 0, -c};
        const double scaled_rate = state.angular_velocity / frequency_;
        PendulumObservables result{
            pivot_W_ + direction * length_,
            math::Vector3{c, 0, s} * (length_ * state.angular_velocity),
            direction,
            weight_ * (c + scaled_rate * scaled_rate),
            energy_scale_ * dimensionlessEnergy(state)};
        if (!result.position_W.isFinite() || !result.velocity_W.isFinite()
            || !std::isfinite(result.energy) || !std::isfinite(result.tension) || result.tension <= 0) {
            throw std::overflow_error("Pendulum observables are not representable with positive tension");
        }
        if (((result.position_W - pivot_W_) / length_ - direction).norm() > 1e-10) {
            throw std::overflow_error("World coordinates cannot resolve pendulum geometry accurately");
        }
        return result;
    }

private:
    double dimensionlessEnergy(const PendulumState& state) const {
        const double speed = state.angular_velocity / frequency_;
        const double half_sine = std::sin(state.angle * 0.5);
        // Avoid cancellation of 1-cos(angle) for small amplitudes.
        return 0.5 * speed * speed + 2 * half_sine * half_sine;
    }
    double length_, mass_, gravity_;
    math::Vector3 pivot_W_;
    double frequency_squared_, frequency_, weight_, energy_scale_;
};

} // namespace csim::dynamics
