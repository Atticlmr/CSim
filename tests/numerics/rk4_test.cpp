#include <csim/numerics/rk4.hpp>
#include <csim/math/matrix.hpp>
#include <csim/math/quaternion.hpp>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using csim::numerics::rk4Step;
using csim::math::Quaternion;
using csim::math::Vector3;

void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance = 1e-12) {
    check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
          "expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}
template <typename Error, typename Function>
void throws(Function&& function) {
    try { function(); } catch (const Error&) { return; }
    throw std::runtime_error("expected exception was not raised");
}

void stageContract() {
    struct Control { double value; } control{3.0};
    struct Dynamics {
        const Control* expected;
        int calls = 0;
        explicit Dynamics(const Control* c) : expected(c) {}
        Dynamics(const Dynamics&) = delete;
        double operator()(double t, const double& x, const Control& u) & {
            constexpr double times[]{1.0, 1.25, 1.25, 1.5};
            constexpr double states[]{2.0, 3.5, 3.9375, 6.09375};
            check(calls < 4 && &u == expected, "control identity/call count");
            near(t, times[calls]);
            near(x, states[calls]);
            ++calls;
            return t + x + u.value;
        }
    } dynamics{&control};
    const double initial = 2.0;
    near(rk4Step(1.0, initial, control, 0.5, dynamics), 6.0390625);
    check(dynamics.calls == 4, "four stages on the same noncopyable callable");
    near(initial, 2.0);
    near(control.value, 3.0);
    near(rk4Step(1.0, initial, control, 0.5, Dynamics{&control}), 6.0390625);

    // Simpson-equivalent quadrature is exact for this cubic time-dependent RHS.
    near(rk4Step(2.0, -3.0, 0, 1.0,
                 [](double t, double, int) { return t * t * t; }), 13.25);
}

void decayConvergence() {
    double previous = 0;
    std::cout << "decay: dt, absolute error\n";
    for (int steps : {5, 10, 20}) {
        const double dt = 1.0 / steps;
        double x = 1.0;
        for (int i = 0; i < steps; ++i) {
            x = rk4Step(i * dt, x, 2.0, dt,
                        [](double, double value, double rate) { return -rate * value; });
        }
        const double error = std::abs(x - std::exp(-2.0));
        check(error > 1e-9 && error < 1e-4, "decay error range");
        if (previous != 0) check(previous / error > 14 && previous / error < 23,
                                 "fourth-order decay convergence");
        previous = error;
        std::cout << std::setprecision(10) << dt << ", " << error << '\n';
    }
}

// Explicitly not default-constructible: RK4 needs no zero accumulator or +=.
struct Particle {
    Vector3 position, velocity;
    Particle() = delete;
    Particle(Vector3 p, Vector3 v) : position(p), velocity(v) {}
    bool isFinite() const { return position.isFinite() && velocity.isFinite(); }
};
Particle operator+(const Particle& a, const Particle& b) {
    return {a.position + b.position, a.velocity + b.velocity};
}
Particle operator*(const Particle& a, double scalar) {
    return {a.position * scalar, a.velocity * scalar};
}

void freeFallAndStateTypes() {
    const Particle initial{{1, -2, 20}, {2, -3, 4}};
    Particle state = initial;
    const Vector3 gravity{0, 0, -9.80665};
    for (int i = 0; i < 100; ++i) {
        state = rk4Step(i * 0.01, state, gravity, 0.01,
                       [](double, const Particle& s, const Vector3& g) -> Particle {
                           return {s.velocity, g};
                       });
    }
    near((state.position - (initial.position + initial.velocity + gravity * 0.5)).norm(), 0);
    near((state.velocity - (initial.velocity + gravity)).norm(), 0);
    near(initial.position.z, 20);

    using Column = csim::math::Matrix<2, 1>;
    const Column column{1, 2};
    const auto evolved = rk4Step(0, column, 0, 0.1,
        [](double, const Column& x, int) { return x; });
    near(evolved(0, 0), 1.1051708333333333);
    near(evolved(1, 0), 2 * evolved(0, 0));
    const auto vector = rk4Step(0, Vector3{1, 2, 3}, gravity, 0.1,
        [](double, const Vector3&, const Vector3& g) { return g; });
    near(vector.z, 3 - 0.980665);
}

double attitudeError(const Quaternion& a, const Quaternion& b) {
    const Quaternion delta = a.normalized().conjugated() * b.normalized();
    return 2 * std::atan2(std::hypot(delta.x, delta.y, delta.z), std::abs(delta.w));
}

void attitudeConvergence() {
    const Quaternion initial = Quaternion::fromRollPitchYaw(0.7, 0.2, -0.4);
    const Vector3 omega{0.4, -0.6, 0.8};
    const Quaternion exact = initial * Quaternion::fromAxisAngle(omega, 2 * omega.norm());
    const auto rhs = [](double, const Quaternion& q, const Vector3& w) {
        return q.derivativeBodyRate(w);
    };
    double previous = 0;
    std::cout << "attitude: dt, raw norm drift, raw angle error, projected angle error\n";
    for (int steps : {10, 20, 40}) {
        const double dt = 2.0 / steps;
        Quaternion raw = initial;
        Quaternion projected = initial;
        for (int i = 0; i < steps; ++i) {
            raw = rk4Step(i * dt, raw, omega, dt, rhs);
            projected = rk4Step(i * dt, projected, omega, dt, rhs).normalized();
        }
        const double error = attitudeError(raw, exact);
        const double projected_error = attitudeError(projected, exact);
        check(error > 1e-10 && error < 1e-5, "attitude error range");
        if (previous != 0) check(previous / error > 15 && previous / error < 17,
                                 "fourth-order attitude convergence");
        near(projected.norm(), 1, 4e-16);
        near(error, projected_error, 2e-15);
        previous = error;
        std::cout << dt << ", " << std::abs(raw.norm() - 1) << ", "
                  << error << ", " << projected_error << '\n';
    }
    // Zero derivative must be zero components, not Quaternion{} (identity).
    const Quaternion scaled = initial * 3;
    const Quaternion stationary = rk4Step(0, scaled, Vector3{}, 0.1, rhs);
    near((stationary - scaled).norm(), 0);
    near(stationary.norm(), 3);
    const auto negative = rk4Step(0, -initial, omega, 0.1, rhs);
    const auto positive = rk4Step(0, initial, omega, 0.1, rhs);
    near((negative + positive).norm(), 0);
    const Quaternion raw_scaled = rk4Step(0, scaled, omega, 0.1, rhs);
    near((raw_scaled - positive * 3).norm(), 0);
}

void invalidTimeAndInput() {
    int calls = 0;
    auto rhs = [&](double, double, int) { ++calls; return 0.0; };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double dt : {0.0, -0.0, -0.1, nan, inf, -inf}) {
        throws<std::invalid_argument>([&] { rk4Step(0, 1.0, 0, dt, rhs); });
    }
    for (double value : {nan, inf, -inf}) {
        throws<std::invalid_argument>([&] { rk4Step(value, 1.0, 0, 0.1, rhs); });
        throws<std::invalid_argument>([&] { rk4Step(0, value, 0, 0.1, rhs); });
    }
    const double max = std::numeric_limits<double>::max();
    for (auto times : {std::array<double, 2>{max, max},
                       std::array<double, 2>{0x1p53, 1},
                       std::array<double, 2>{-0x1p53, 1},
                       std::array<double, 2>{0, std::numeric_limits<double>::denorm_min()}}) {
        throws<std::overflow_error>([&] { rk4Step(times[0], 1.0, 0, times[1], rhs); });
    }
    check(calls == 0, "invalid time/input rejected before RHS");
    throws<std::invalid_argument>([&] {
        rk4Step(0, Vector3{0, nan, 1}, 0, 0.1,
                [](double, const Vector3&, int) { return Vector3{}; });
    });
    near(rk4Step(-1, 1.0, 0, 0.25, rhs), 1);
}

void numericalFailures() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double max = std::numeric_limits<double>::max();
    for (double bad : {nan, inf, -inf}) {
        for (int failed_stage = 1; failed_stage <= 4; ++failed_stage) {
            int calls = 0;
            throws<std::overflow_error>([&] {
                rk4Step(0, 1.0, 0, 0.1, [&](double, double, int) {
                    return ++calls == failed_stage ? bad : 0.0;
                });
            });
            check(calls == failed_stage, "abort on non-finite derivative");
        }
    }
    // Finite derivatives, overflowing stage state or final result.
    for (int failed_stage = 1; failed_stage <= 4; ++failed_stage) {
        int calls = 0;
        const double initial = max;
        throws<std::overflow_error>([&] {
            rk4Step(0, initial, 0, 2.0, [&](double, double, int) {
                return ++calls == failed_stage ? max : 0.0;
            });
        });
        check(calls == failed_stage, "abort on overflowing stage/result");
        near(initial, max, 0);
    }
    // Weighted sum must avoid overflowing the unscaled sum of large derivatives.
    const double large = rk4Step(0, 0.0, 0, 0.1,
                                [=](double, double, int) { return max / 2; });
    near(large / max, 0.05, 1e-16);
    const double tiny_dt = 2 * std::numeric_limits<double>::denorm_min();
    const double tiny_step = rk4Step(0, 0.0, 0, tiny_dt,
                                    [=](double, double, int) { return max / 2; });
    near(tiny_step / ((max / 2) * tiny_dt), 1, 5e-16);
}

void modelException() {
    struct ModelError : std::runtime_error { using std::runtime_error::runtime_error; };
    const Vector3 initial{1, 2, 3};
    int calls = 0;
    throws<ModelError>([&] {
        rk4Step(0, initial, 0, 0.1, [&](double, const Vector3&, int) -> Vector3 {
            if (++calls == 3) throw ModelError("model outside valid domain");
            return {1, 1, 1};
        });
    });
    check(calls == 3, "propagate model error without further evaluations");
    near((initial - Vector3{1, 2, 3}).norm(), 0);
}
} // namespace

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]{
        {"stage and control contract", stageContract},
        {"decay convergence", decayConvergence},
        {"free fall and generic states", freeFallAndStateTypes},
        {"attitude convergence and normalization", attitudeConvergence},
        {"invalid time and input", invalidTimeAndInput},
        {"non-finite stages and overflow", numericalFailures},
        {"model exception propagation", modelException},
    };
    int failures = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& e) {
            ++failures;
            std::cerr << "FAIL: " << test.name << ": " << e.what() << '\n';
        }
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
