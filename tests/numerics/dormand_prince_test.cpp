#include <csim/numerics/dormand_prince.hpp>
#include <csim/math/quaternion.hpp>

#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace csim::numerics;
using csim::math::Quaternion;
using csim::math::Vector3;
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance = 1e-12) {
    check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, "numeric mismatch");
}
template <typename Error, typename Function>
void throws(Function&& f) {
    try { f(); } catch (const Error&) { return; }
    throw std::runtime_error("expected exception missing");
}
const auto growth = [](double, double x, double rate) { return rate * x; };

void trialContract() {
    struct Control { double speed; } control{3};
    struct Dynamics {
        const Control* expected;
        int calls = 0;
        explicit Dynamics(const Control* p) : expected(p) {}
        Dynamics(const Dynamics&) = delete;
        double operator()(double t, const double& x, const Control& u) & {
            const double c[]{0, 0.2, 0.3, 0.8, 8.0/9.0, 1, 1};
            check(calls < 7 && &u == expected, "same callable and control");
            near(t, 2 + c[calls] * 0.5);
            near(x, 1 + 3 * c[calls] * 0.5);
            ++calls;
            return u.speed;
        }
    } rhs{&control};
    const auto trial = dormandPrince54Trial(2, 1.0, control, 0.5, rhs);
    near(trial.state, 2.5);
    near(trial.error, 0);
    check(rhs.calls == 7, "seven evaluations");
    const auto golden = dormandPrince54Trial(0, 1.0, 1.0, 1, growth);
    // Exact rational results for the embedded pair, including the x5-x4 sign.
    near(golden.state, 1631.0 / 600.0);
    near(golden.state - golden.error, 326263.0 / 120000.0);
    near(golden.error, -21.0 / 40000.0, 1e-14);
    const auto timed = dormandPrince54Trial(0.4, -1.0, 0, 0.7,
        [](double t, double, int) { return t*t*t*t; });
    near(timed.state, -1 + (std::pow(1.1,5) - std::pow(0.4,5))/5);
}

void trialOrders() {
    double previous = 0;
    std::cout << "fixed DP5: h, global error\n";
    for (int count : {4, 8, 16}) {
        const double h = 1.0 / count;
        double x = 1;
        for (int i = 0; i < count; ++i) x = dormandPrince54Trial(i*h, x, 1.0, h, growth).state;
        const double error = std::abs(x - std::exp(1.0));
        check(error > 1e-12, "error above roundoff");
        if (previous != 0) check(previous/error > 24 && previous/error < 40, "global order five");
        previous = error;
        std::cout << h << ", " << std::setprecision(10) << error << '\n';
    }
    previous = 0;
    for (double h : {0.5, 0.25, 0.125}) {
        const double error = std::abs(dormandPrince54Trial(0, 1.0, 1.0, h, growth).error);
        if (previous != 0) check(previous/error > 27 && previous/error < 35, "estimator order five");
        previous = error;
    }
}

void toleranceAndBoundary() {
    double previous_error = 1;
    std::size_t previous_calls = 0;
    for (double tolerance : {1e-4, 1e-7, 1e-10}) {
        DormandPrince54<double> solver(0, 1, 1);
        const ScalarErrorNorm norm(tolerance, tolerance * 0.01);
        std::size_t calls = 0, accepted = 0, rejected = 0;
        while (solver.time() < 2) {
            const double old_time = solver.time();
            const auto report = solver.step(-2.0, [&](double, double x, double r) {
                ++calls; return r*x;
            }, norm, 2);
            check(++accepted < 10000, "bounded integration loop");
            check(solver.time() > old_time && solver.time() <= 2, "forward bounded step");
            check(report.error_norm <= 1 && report.attempts == report.rejected + 1, "acceptance report");
            rejected += report.rejected;
        }
        near(solver.time(), 2, 0);
        const double error = std::abs(solver.state() - std::exp(-4.0));
        check(error < previous_error * 0.1, "tighter tolerance improves solution");
        check(calls > previous_calls && calls == 7*(accepted+rejected), "work accounting without FSAL");
        std::cout << "rtol=" << tolerance << ", error=" << error << ", RHS=" << calls << '\n';
        previous_error = error;
        previous_calls = calls;
    }
    DormandPrince54<double> short_step(0, 0, 0.1, {0.01, 0.1, 8});
    const auto constant = [](double, double, int) { return 1.0; };
    auto report = short_step.step(0, constant, ScalarErrorNorm{}, 0.003);
    near(short_step.time(), 0.003, 0);
    near(short_step.state(), 0.003);
    near(report.step_size, 0.003, 0);
    check(short_step.nextStepSize() >= 0.01 && short_step.nextStepSize() <= 0.1, "next step clamped");
}

void rejectionAndReset() {
    DormandPrince54<double> solver(0, 1, 0.5);
    int norms = 0;
    auto scripted_norm = [&](const double& old, const double&, const double&) {
        near(old, 1);
        near(solver.time(), 0, 0);
        near(solver.state(), 1, 0);
        near(solver.nextStepSize(), 0.5, 0);
        return ++norms == 1 ? 32.0 : 0.0;
    };
    const auto report = solver.step(1.0, growth, scripted_norm, 1);
    check(report.attempts == 2 && report.rejected == 1, "reject then accept");
    check(report.step_size < 0.5 && solver.nextStepSize() <= report.step_size, "avoid growth after rejection");
    solver.reset(-1, 2, 0.1);
    near(solver.time(), -1, 0);
    near(solver.state(), 2, 0);
    near(solver.nextStepSize(), 0.1, 0);
    // Changed controls must take effect immediately: no cached derivative.
    const auto constant = [](double, double, double u) { return u; };
    solver.step(0.0, constant, ScalarErrorNorm{}, -0.9);
    solver.step(2.0, constant, ScalarErrorNorm{}, -0.8);
    near(solver.state(), 2.2);
}

void failuresAreAtomic() {
    auto unchanged = [](const DormandPrince54<double>& s) {
        near(s.time(), 0, 0); near(s.state(), 1, 0); near(s.nextStepSize(), 0.1, 0);
    };
    DormandPrince54<double> solver(0, 1, 0.1, {0.001, 1, 2});
    int calls = 0;
    throws<IntegrationFailure>([&] {
        solver.step(0, [&](double, double, int) { ++calls; return 0.0; },
                    [](double, double, double) { return 2.0; }, 1);
    });
    check(calls == 14, "attempt budget");
    unchanged(solver);
    DormandPrince54<double> floor(0, 1, 0.1, {0.1, 1, 5});
    throws<IntegrationFailure>([&] {
        floor.step(1.0, growth, [](double, double, double) { return std::numeric_limits<double>::infinity(); }, 1);
    });
    unchanged(floor);
    for (double bad : {-1.0, -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        throws<std::invalid_argument>([&] {
            solver.step(1.0, growth, [=](double, double, double) { return bad; }, 1);
        });
        unchanged(solver);
    }
    struct ModelError : std::runtime_error { using std::runtime_error::runtime_error; };
    throws<ModelError>([&] {
        solver.step(0, [](double, double, int) -> double { throw ModelError("invalid physical domain"); },
                    ScalarErrorNorm{}, 1);
    });
    unchanged(solver);
    throws<ModelError>([&] {
        solver.step(1.0, growth, [](double, double, double) -> double { throw ModelError("norm failure"); }, 1);
    });
    unchanged(solver);
    throws<std::overflow_error>([&] {
        solver.step(0, [](double, double, int) { return std::numeric_limits<double>::infinity(); },
                    ScalarErrorNorm{}, 1);
    });
    unchanged(solver);
    throws<std::invalid_argument>([&] { solver.reset(1, 2, -0.1); });
    unchanged(solver);
    solver.step(0.0, growth, ScalarErrorNorm{}, 0.1);
    near(solver.time(), 0.1, 0); // Recovery after failures.
}

void genericAndAttitude() {
    struct Value {
        double x;
        Value() = delete;
        explicit Value(double v) : x(v) {}
        bool isFinite() const { return std::isfinite(x); }
        Value operator+(const Value& b) const { return Value(x+b.x); }
        Value operator*(double s) const { return Value(x*s); }
    };
    const auto trial = dormandPrince54Trial(0, Value(1), 0, 0.1,
        [](double, const Value& x, int) { return x; });
    near(trial.state.x, std::exp(0.1), 3e-10);
    const auto q0 = Quaternion::fromRollPitchYaw(0.4, -0.2, 0.8);
    const Vector3 omega{0.4, -0.6, 0.8};
    DormandPrince54<Quaternion> solver(0, q0, 0.2);
    auto norm = [](const Quaternion&, const Quaternion&, const Quaternion& e) { return e.norm()/1e-10; };
    auto rhs = [](double, const Quaternion& q, const Vector3& w) { return q.derivativeBodyRate(w); };
    int count = 0;
    while (solver.time() < 2) {
        solver.step(omega, rhs, norm, 2);
        check(++count < 1000, "attitude step budget");
    }
    const auto exact = q0 * Quaternion::fromAxisAngle(omega, 2*omega.norm());
    check((solver.state()-exact).norm() < 1e-9, "constant-rate attitude exact reference");
    const auto zero = dormandPrince54Trial(0, q0*3, Vector3{}, 0.1, rhs);
    near((zero.state-q0*3).norm(), 0);
    near(zero.error.norm(), 0, 0); // Error is a zero quaternion, never identity.
}

void invalidInputs() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double value : {nan, inf, -inf}) {
        throws<std::invalid_argument>([&] { dormandPrince54Trial(value, 1.0, 1.0, 0.1, growth); });
        throws<std::invalid_argument>([&] { dormandPrince54Trial(0, value, 1.0, 0.1, growth); });
        throws<std::invalid_argument>([&] { ScalarErrorNorm norm(value, 1e-9); });
        throws<std::invalid_argument>([&] { ScalarErrorNorm norm(1e-6, value); });
    }
    for (double dt : {0.0, -0.1, nan, inf}) {
        throws<std::invalid_argument>([&] { dormandPrince54Trial(0, 1.0, 1.0, dt, growth); });
    }
    throws<std::invalid_argument>([] { ScalarErrorNorm norm(0, 0); });
    throws<std::invalid_argument>([] { DormandPrince54<double> s(0, 1, 0.1, {0, 1, 5}); });
    throws<std::invalid_argument>([] { DormandPrince54<double> s(0, 1, 0.1, {0.2, 0.1, 5}); });
    throws<std::invalid_argument>([] { DormandPrince54<double> s(0, 1, 0.1, {0.001, 1, 0}); });
    DormandPrince54<double> s(1, 1, 0.1);
    for (double bound : {1.0, 0.0, nan, inf}) {
        throws<std::invalid_argument>([&] { s.step(1.0, growth, ScalarErrorNorm{}, bound); });
    }
    near(ScalarErrorNorm(0, 0.5)(1, 2, -0.25), 0.5);
    near(ScalarErrorNorm(0.1, 0.1)(1, 2, -0.3), 1);
}

void stageFailures() {
    for (int bad_stage = 1; bad_stage <= 7; ++bad_stage) {
        int calls = 0;
        throws<std::overflow_error>([&] {
            dormandPrince54Trial(0, 1.0, 0, 0.1, [&](double, double, int) {
                return ++calls == bad_stage ? std::numeric_limits<double>::quiet_NaN() : 0.0;
            });
        });
        check(calls == bad_stage, "stop at invalid derivative");
    }
    const double maximum = std::numeric_limits<double>::max();
    throws<std::overflow_error>([&] {
        dormandPrince54Trial(0, maximum, 0, 1, [=](double, double, int) { return maximum; });
    });
    int calls = 0;
    auto rhs = [&](double, double, int) { ++calls; return 0.0; };
    throws<std::overflow_error>([&] { dormandPrince54Trial(0x1p53, 1.0, 0, 1, rhs); });
    throws<std::overflow_error>([&] { dormandPrince54Trial(maximum, 1.0, 0, maximum, rhs); });
    throws<std::overflow_error>([&] {
        dormandPrince54Trial(0, 1.0, 0, std::numeric_limits<double>::denorm_min(), rhs);
    });
    check(calls == 0, "validate all stage times before RHS");
}
} // namespace
int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]{
        {"trial stages and embedded sign", trialContract}, {"method and estimator orders", trialOrders},
        {"adaptive tolerance and time boundary", toleranceAndBoundary},
        {"rejection, reset and changed control", rejectionAndReset},
        {"atomic failures and recovery", failuresAreAtomic}, {"generic state and attitude", genericAndAttitude},
        {"invalid options and input", invalidInputs}, {"non-finite stages and time precision", stageFailures}
    };
    int failures = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& e) { ++failures; std::cerr << "FAIL: " << test.name << ": " << e.what() << '\n'; }
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
