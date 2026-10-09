#pragma once

#include <csim/numerics/integration_detail.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <utility>

namespace csim::numerics {

// No default-zero assumption: Quaternion{} represents identity, not zero error.
template <typename State>
struct EmbeddedStep {
    State state; // Fifth-order candidate, not yet accepted.
    State error; // Signed x5 - x4, computed directly from derivative weights.
};

// Seven RHS evaluations; the last two share the endpoint time, not the state.
// No FSAL cache, projection, acceptance or mutation of input. See docs/dormand-prince.md.
template <typename State, typename Control, typename Dynamics>
EmbeddedStep<State> dormandPrince54Trial(double t, const State& state,
                                       const Control& control, double dt,
                                       Dynamics&& dynamics) {
    if (!std::isfinite(t) || !std::isfinite(dt) || dt <= 0
        || !detail::stateIsFinite(state)) {
        throw std::invalid_argument("Dormand-Prince requires finite state/time and positive finite dt");
    }
    const std::array<double, 6> times{t, t + dt / 5, t + dt * (3.0 / 10.0),
                                    t + dt * (4.0 / 5.0), t + dt * (8.0 / 9.0), t + dt};
    for (std::size_t i = 1; i < times.size(); ++i) {
        if (!std::isfinite(times[i]) || !(times[i] > times[i - 1])) {
            throw std::overflow_error("Dormand-Prince stage times are not distinct finite values");
        }
    }
    auto evaluate = [&](double time, const State& stage) {
        detail::checkComputedState(stage);
        const State derivative = dynamics(time, stage, control);
        detail::checkComputedState(derivative);
        return derivative;
    };
    // Dormand-Prince tableau; references and the error sign convention are in docs.
    const State k1 = evaluate(times[0], state);
    const State k2 = evaluate(times[1], state + (k1 * (1.0 / 5.0)) * dt);
    const State k3 = evaluate(times[2], state + (k1 * (3.0 / 40.0) + k2 * (9.0 / 40.0)) * dt);
    const State k4 = evaluate(times[3], state + (k1 * (44.0 / 45.0) + k2 * (-56.0 / 15.0)
                                              + k3 * (32.0 / 9.0)) * dt);
    const State k5 = evaluate(times[4], state + (k1 * (19372.0 / 6561.0) + k2 * (-25360.0 / 2187.0)
                                              + k3 * (64448.0 / 6561.0) + k4 * (-212.0 / 729.0)) * dt);
    const State k6 = evaluate(times[5], state + (k1 * (9017.0 / 3168.0) + k2 * (-355.0 / 33.0)
                                              + k3 * (46732.0 / 5247.0) + k4 * (49.0 / 176.0)
                                              + k5 * (-5103.0 / 18656.0)) * dt);
    State candidate = state + (k1 * (35.0 / 384.0) + k3 * (500.0 / 1113.0)
                               + k4 * (125.0 / 192.0) + k5 * (-2187.0 / 6784.0)
                               + k6 * (11.0 / 84.0)) * dt;
    const State k7 = evaluate(times[5], candidate);
    State error = (k1 * (71.0 / 57600.0) + k3 * (-71.0 / 16695.0)
                   + k4 * (71.0 / 1920.0) + k5 * (-17253.0 / 339200.0)
                   + k6 * (22.0 / 525.0) + k7 * (-1.0 / 40.0)) * dt;
    detail::checkComputedState(error);
    return {std::move(candidate), std::move(error)};
}

struct AdaptiveStepOptions {
    double min_step = 1e-12;
    double max_step = 1.0;
    std::size_t max_attempts = 32; // Includes the successful attempt, if any.
};

struct AdaptiveStepReport {
    double step_size;
    double error_norm;
    std::size_t attempts;
    std::size_t rejected;
};

class IntegrationFailure : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Scalar-only policy. Composite/physical states must supply their own norm with
// per-component units/tolerances and, where needed, a geometry-aware error metric.
class ScalarErrorNorm {
public:
    explicit ScalarErrorNorm(double relative_tolerance = 1e-6, double absolute_tolerance = 1e-9)
        : rtol_(relative_tolerance), atol_(absolute_tolerance) {
        if (!std::isfinite(rtol_) || rtol_ < 0 || !std::isfinite(atol_) || atol_ <= 0) {
            throw std::invalid_argument("Error tolerances require finite rtol >= 0 and atol > 0");
        }
    }
    double operator()(double old_state, double candidate, double error) const {
        if (!std::isfinite(old_state) || !std::isfinite(candidate) || !std::isfinite(error)) {
            throw std::invalid_argument("Scalar error inputs must be finite");
        }
        const long double scale = static_cast<long double>(atol_)
            + static_cast<long double>(rtol_) * std::max(std::abs(old_state), std::abs(candidate));
        return static_cast<double>(std::abs(static_cast<long double>(error)) / scale);
    }
private:
    double rtol_, atol_;
};

// Owns accepted state/time and the proposed next step. Only forward integration.
// All failures leave these three values unchanged. User callback side effects
// cannot be rolled back. State must have value semantics and no-throw move assignment.
template <typename State>
class DormandPrince54 {
    static_assert(std::is_nothrow_move_assignable_v<State>,
                  "Adaptive state needs no-throw move assignment for atomic acceptance");
public:
    DormandPrince54(double time, State initial, double first_step,
                   AdaptiveStepOptions options = {})
        : time_(time), state_(std::move(initial)), next_step_(first_step), options_(options) {
        if (!std::isfinite(options_.min_step) || options_.min_step <= 0
            || !std::isfinite(options_.max_step) || options_.max_step < options_.min_step
            || options_.max_attempts == 0) {
            throw std::invalid_argument("Invalid adaptive step limits");
        }
        validateInitial(time_, state_, first_step);
    }

    double time() const noexcept { return time_; }
    const State& state() const noexcept { return state_; }
    double nextStepSize() const noexcept { return next_step_; }

    void reset(double time, State initial, double first_step) {
        validateInitial(time, initial, first_step);
        state_ = std::move(initial);
        time_ = time;
        next_step_ = first_step;
    }

    // norm(old, candidate, signed_error) -> nonnegative dimensionless error.
    // <=1 accepts; +Inf rejects; NaN/negative norms are invalid. Each call accepts
    // at most one internal step and never crosses time_bound (control/output boundary).
    template <typename Control, typename Dynamics, typename ErrorNorm>
    AdaptiveStepReport step(const Control& control, Dynamics&& dynamics,
                            ErrorNorm&& norm, double time_bound) {
        if (!std::isfinite(time_bound) || time_bound <= time_) {
            throw std::invalid_argument("Time bound must be finite and after accepted time");
        }
        const double remaining = time_bound - time_;
        if (!std::isfinite(remaining)) {
            throw std::overflow_error("Time interval is not representable");
        }
        double h = std::min(next_step_, remaining);
        for (std::size_t attempt = 0; attempt < options_.max_attempts; ++attempt) {
            // A final boundary remainder may be below min_step, but cannot be
            // subdivided further if rejected. Use the actual representable increment.
            const double endpoint = h >= remaining ? time_bound : std::min(time_ + h, time_bound);
            h = endpoint - time_;
            if (h <= 0) throw IntegrationFailure("Adaptive time cannot advance");
            if (time_ + h != endpoint) {
                throw IntegrationFailure("Boundary increment cannot reproduce endpoint time");
            }
            auto trial = dormandPrince54Trial(time_, state_, control, h, dynamics);
            const double error = norm(std::as_const(state_), std::as_const(trial.state),
                                      std::as_const(trial.error));
            if (std::isnan(error) || error < 0) {
                throw std::invalid_argument("Error norm must be nonnegative and not NaN");
            }
            double factor = error == 0 ? 5.0 : std::clamp(0.9 * std::pow(error, -0.2), 0.2, 5.0);
            if (error <= 1) {
                if (attempt != 0) factor = std::min(factor, 1.0);
                const double suggested = h > options_.max_step / factor ? options_.max_step
                    : std::clamp(h * factor, options_.min_step, options_.max_step);
                state_ = std::move(trial.state);
                time_ = endpoint;
                next_step_ = suggested;
                return {h, error, attempt + 1, attempt};
            }
            if (h <= options_.min_step) {
                throw IntegrationFailure("Error tolerance cannot be met at minimum step");
            }
            const double smaller = std::max(options_.min_step, h * factor);
            if (!(smaller < h)) throw IntegrationFailure("Adaptive step cannot shrink");
            h = smaller;
        }
        throw IntegrationFailure("Adaptive step attempt limit exceeded");
    }

private:
    void validateInitial(double time, const State& state, double first_step) const {
        if (!std::isfinite(time) || !detail::stateIsFinite(state)
            || !std::isfinite(first_step) || first_step < options_.min_step
            || first_step > options_.max_step) {
            throw std::invalid_argument("Invalid adaptive initial state, time or first step");
        }
    }
    double time_;
    State state_;
    double next_step_;
    AdaptiveStepOptions options_;
};

// TODO: Dense output and optional FSAL reuse require explicit cache invalidation
//       on state/control/model changes. Neither is provided by this first version.

} // namespace csim::numerics
