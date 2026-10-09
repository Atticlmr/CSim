#pragma once

#include <csim/math/lu.hpp>
#include <csim/dynamics/aerodynamics.hpp>
#include <csim/math/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace csim::dynamics {

// W: right handed, Z up. B: forward/left/up, origin at the centre of mass.
struct DroneState {
    math::Vector3 position_W{};
    math::Vector3 velocity_W{};
    math::Quaternion q_WB{}; // Hamilton wxyz; B -> W. Identity only for initial state.
    math::Vector3 angular_velocity_B{};

    bool isFinite() const {
        return position_W.isFinite() && velocity_W.isFinite() && q_WB.isFinite()
            && angular_velocity_B.isFinite();
    }
};

// Raw state arithmetic for C++ integrators: never normalize a derivative.
inline DroneState operator+(const DroneState& a, const DroneState& b) noexcept {
    return {a.position_W + b.position_W, a.velocity_W + b.velocity_W,
            a.q_WB + b.q_WB, a.angular_velocity_B + b.angular_velocity_B};
}
inline DroneState operator*(const DroneState& state, double scalar) noexcept {
    return {state.position_W * scalar, state.velocity_W * scalar,
            state.q_WB * scalar, state.angular_velocity_B * scalar};
}

// Actual applied thrust and body torque, not a feedback controller or command.
struct DroneControl {
    double thrust = 0; // N, along +Z_B, nonnegative; held for a full physical step.
    math::Vector3 torque_B{}; // N m, about the centre of mass.
};

struct DroneObservables {
    math::Vector3 acceleration_W;
    math::Vector3 angular_acceleration_B;
    math::Vector3 thrust_W;
    math::Vector3 angular_momentum_W; // About the centre of mass, expressed in W.
    double energy; // Translational + rotational kinetic + m*g*z, reference z=0.
    AirLoads aerodynamics;
};

class Drone {
public:
    static constexpr math::Matrix3 defaultInertia() noexcept {
        return math::Matrix3{0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.04};
    }

    Drone(double mass = 1, math::Matrix3 inertia_B = defaultInertia(), double gravity = 9.80665, DragConfig drag = {}, WindField wind = {})
        : mass_(positive(mass)), gravity_(positive(gravity)),
          inertia_B_(validatedInertia(inertia_B)), inertia_solver_(inertia_B_), drag_(std::move(drag)), wind_(std::move(wind)) {
        drag_.validate(false); wind_.validate();
    }

    const DragConfig& drag() const noexcept { return drag_; }
    const WindField& wind() const noexcept { return wind_; }
    AirLoads airLoads(const DroneState& state,double time=0) const {
        return drag_.forces(state.velocity_W,wind_.at(state.position_W,time));
    }
    double mass() const noexcept { return mass_; }
    double gravity() const noexcept { return gravity_; }
    const math::Matrix3& inertia() const noexcept { return inertia_B_; }

    static void validateControl(const DroneControl& control) {
        if (!std::isfinite(control.thrust) || control.thrust < 0 || !control.torque_B.isFinite()) {
            throw std::invalid_argument("Drone requires finite nonnegative thrust and finite body torque");
        }
    }

    // Used at public state boundaries. Accept any finite nonzero quaternion and
    // normalize a copy, including scaled and opposite-sign representations.
    static DroneState normalizedState(DroneState state) {
        if (!state.isFinite()) throw std::invalid_argument("Drone state must be finite");
        state.q_WB = state.q_WB.normalized();
        return state;
    }

    DroneState derivative(const DroneState& state, const DroneControl& control, double time=0) const {
        if (!state.isFinite()) throw std::invalid_argument("Drone state must be finite");
        validateControl(control);
        // Rotation uses a normalized copy at RK stages; qdot uses raw stage q.
        const double thrust_acceleration = control.thrust / mass_;
        if (!std::isfinite(thrust_acceleration)) {
            throw std::overflow_error("Drone thrust acceleration is not representable");
        }
        auto acceleration = state.q_WB.rotate({0, 0, thrust_acceleration})
            + math::Vector3{0, 0, -gravity_};
        if (drag_.enabled()) acceleration += airLoads(state,time).total_force_W/mass_;
        const auto momentum_B = inertia_B_ * state.angular_velocity_B;
        const auto rhs = control.torque_B - state.angular_velocity_B.cross(momentum_B);
        if (!momentum_B.isFinite() || !rhs.isFinite()) {
            throw std::overflow_error("Drone rotational dynamics overflow");
        }
        const DroneState result{state.velocity_W, acceleration,
            state.q_WB.derivativeBodyRate(state.angular_velocity_B), inertia_solver_.solve(rhs)};
        if (!result.isFinite()) throw std::overflow_error("Drone derivative is not representable");
        return result;
    }

    DroneObservables observe(const DroneState& state, const DroneControl& control, double time=0) const {
        const auto rate = derivative(state, control, time);
        const auto momentum_B = inertia_B_ * state.angular_velocity_B;
        const double kinetic = 0.5 * mass_ * state.velocity_W.squaredNorm()
            + 0.5 * state.angular_velocity_B.dot(momentum_B);
        const DroneObservables result{rate.velocity_W, rate.angular_velocity_B,
            state.q_WB.rotate({0, 0, control.thrust}), state.q_WB.rotate(momentum_B),
            kinetic + mass_ * gravity_ * state.position_W.z,airLoads(state,time)};
        if (!std::isfinite(kinetic) || !std::isfinite(result.energy)) {
            throw std::overflow_error("Drone energy is not representable");
        }
        return result;
    }

private:
    static double positive(double value) {
        if (!std::isfinite(value) || value <= 0) {
            throw std::invalid_argument("Drone mass and gravity must be finite and positive");
        }
        return value;
    }

    static math::Matrix3 validatedInertia(math::Matrix3 matrix) {
        if (!matrix.isFinite()) throw std::invalid_argument("Drone inertia must be finite");
        double scale = 0;
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j) scale = std::max(scale, std::abs(matrix(i, j)));
        if (scale == 0) throw std::invalid_argument("Drone inertia must be positive definite");
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = i + 1; j < 3; ++j) {
                if (std::abs(matrix(i, j) / scale - matrix(j, i) / scale) > 1e-12) {
                    throw std::invalid_argument("Drone inertia must be symmetric");
                }
                // Canonicalize only roundoff-sized asymmetry; expose the actual matrix used.
                matrix(i, j) = matrix(j, i) = matrix(i, j) * 0.5 + matrix(j, i) * 0.5;
            }
        }
        const long double a = static_cast<long double>(matrix(0, 0)) / scale;
        const long double b = static_cast<long double>(matrix(0, 1)) / scale;
        const long double c = static_cast<long double>(matrix(0, 2)) / scale;
        const long double d = static_cast<long double>(matrix(1, 1)) / scale;
        const long double e = static_cast<long double>(matrix(1, 2)) / scale;
        const long double f = static_cast<long double>(matrix(2, 2)) / scale;
        if (!(a > 0 && a * d - b * b > 0
              && a * (d * f - e * e) - b * (b * f - c * e) + c * (b * e - c * d) > 0)) {
            throw std::invalid_argument("Drone inertia must be positive definite");
        }
        return matrix;
    }

    double mass_;
    double gravity_;
    math::Matrix3 inertia_B_;
    math::PartialPivLU<3> inertia_solver_; // Factor once; solve J*alpha=tau-Omega x J*Omega.
    DragConfig drag_;
    WindField wind_;
};

} // namespace csim::dynamics
