#include <csim/dynamics/pendulum.hpp>
#include <csim/numerics/rk4.hpp>

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

namespace {
using csim::dynamics::Pendulum;
using csim::dynamics::PendulumState;
using csim::math::Vector3;
void check(bool c, const char* msg) { if (!c) throw std::runtime_error(msg); }
void near(double a, double b, double tol = 1e-12) {
    check(std::isfinite(a) && std::abs(a-b) <= tol, "numeric mismatch");
}
template <typename Error, typename F> void throws(F&& f) {
    try { f(); } catch (const Error&) { return; }
    throw std::runtime_error("expected exception missing");
}
PendulumState advance(const Pendulum& p, PendulumState s, double t, double h) {
    return csim::numerics::rk4Step(t, s, 0, h,
        [&](double, const PendulumState& state, int) { return p.derivative(state); });
}

void forcesAndGeometry() {
    const Pendulum p(2, 3, 9.81, {4, -2, 8});
    const auto resting = p.observe({});
    near((resting.position_W-Vector3{4,-2,6}).norm(), 0);
    near(resting.velocity_W.norm(), 0);
    near(resting.tension, 3*9.81);
    near(resting.energy, 0);
    near(p.derivative({}).angle, 0);
    near(p.derivative({}).angular_velocity, 0);
    for (const PendulumState s : {PendulumState{0.4, -0.7}, PendulumState{-0.6, 0.5}}) {
        const auto obs = p.observe(s);
        near((obs.position_W-p.pivot()).norm(), p.length());
        near(obs.cable_direction_W.norm(), 1);
        near(obs.cable_direction_W.dot(obs.velocity_W), 0);
        near(obs.velocity_W.y, 0);
        const Vector3 tangent{std::cos(s.angle), 0, std::sin(s.angle)};
        const Vector3 acceleration = obs.cable_direction_W * (-p.length()*s.angular_velocity*s.angular_velocity)
            + tangent * (p.length()*p.derivative(s).angular_velocity);
        const Vector3 force = Vector3{0,0,-p.mass()*p.gravity()} - obs.cable_direction_W*obs.tension;
        near((acceleration*p.mass()-force).norm(), 0, 2e-14);
        const double independent_energy = 0.5*p.mass()*obs.velocity_W.squaredNorm()
            + p.mass()*p.gravity()*(obs.position_W.z-p.pivot().z+p.length());
        near(obs.energy, independent_energy, 3e-14);
        check(obs.tension > 0, "positive tension");
    }
}

void symmetryAndMass() {
    Pendulum light(1, 1), heavy(1, 7);
    PendulumState a{0.6, -0.3}, b{-0.6, 0.3}, c = a;
    for (int i=0; i<1000; ++i) {
        a = advance(light, a, i*0.002, 0.002);
        b = advance(light, b, i*0.002, 0.002);
        c = advance(heavy, c, i*0.002, 0.002);
    }
    near(a.angle, -b.angle);
    near(a.angular_velocity, -b.angular_velocity);
    near(a.angle, c.angle, 0);
    near(a.angular_velocity, c.angular_velocity, 0);
    near(heavy.observe(c).tension/light.observe(a).tension, 7);
    near(heavy.observe(c).energy/light.observe(a).energy, 7);
}

void smallAmplitudePeriod() {
    const double pi = std::acos(-1.0);
    const Pendulum p;
    const double expected = 2*pi*std::sqrt(p.length()/p.gravity());
    const double h = expected/2000;
    PendulumState s{0.01,0};
    std::vector<double> crossings;
    for (int i=0; i<8000; ++i) {
        const auto next = advance(p, s, i*h, h);
        if (s.angle > 0 && next.angle <= 0) crossings.push_back(i*h+h*s.angle/(s.angle-next.angle));
        s = next;
    }
    check(crossings.size()==4, "one downward zero crossing per cycle");
    const double measured = (crossings.back()-crossings.front())/3;
    check(std::abs(measured/expected-1)<1e-5, "small-amplitude period");
    check(measured > expected, "nonlinear period increases with amplitude");
    std::cout << "small-angle period: reference=" << expected << ", measured=" << measured << '\n';
}

void energyConvergence() {
    const Pendulum p;
    double previous = 1;
    for (double h : {0.04,0.02,0.01}) {
        PendulumState s{0.7,0};
        const double initial_energy = p.observe(s).energy;
        double largest = 0, constraint = 0;
        for (int i=0; i<static_cast<int>(std::round(20/h)); ++i) {
            s = advance(p, s, i*h, h);
            const auto obs = p.observe(s);
            largest = std::max(largest, std::abs(obs.energy-initial_energy)/initial_energy);
            constraint = std::max(constraint, std::abs((obs.position_W-p.pivot()).norm()-p.length()));
        }
        check(largest < previous/10, "energy error decreases with step refinement");
        check(constraint < 1e-14, "geometric cable length");
        if (h==0.01) check(largest<1e-7, "20-second energy error budget");
        previous = largest;
        std::cout << std::setprecision(10) << "h=" << h << ", max relative energy error=" << largest
                  << ", length residual=" << constraint << '\n';
    }
}

void invalidInputs() {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double bad : {0.0,-1.0,inf,nan}) {
        throws<std::invalid_argument>([&] { Pendulum p(bad); });
        throws<std::invalid_argument>([&] { Pendulum p(1,bad); });
        throws<std::invalid_argument>([&] { Pendulum p(1,1,bad); });
    }
    throws<std::invalid_argument>([&] { Pendulum p(1,1,9.81,{0,nan,0}); });
    throws<std::overflow_error>([] { Pendulum p(1,1e308,9.81); });
    throws<std::overflow_error>([] { Pendulum p(1e-308,1,1e308); });
    const Pendulum p;
    for (PendulumState s : {PendulumState{nan,0},PendulumState{0,inf}}) {
        throws<std::invalid_argument>([&] { p.observe(s); });
        throws<std::invalid_argument>([&] { p.derivative(s); });
    }
    for (PendulumState s : {PendulumState{2,0},PendulumState{0,10},PendulumState{std::acos(-1.0)/2,0}}) {
        throws<csim::dynamics::PendulumDomainError>([&] { p.observe(s); });
    }
    const Pendulum distant(1,1,9.81,{1e100,0,1e100});
    throws<std::overflow_error>([&] { distant.observe({0.2,0}); });
}
}
int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]{{"forces and geometry",forcesAndGeometry},{"symmetry and mass",symmetryAndMass},
        {"small-amplitude period",smallAmplitudePeriod},{"energy convergence",energyConvergence},
        {"invalid parameters and model domain",invalidInputs}};
    int failed=0;
    for (const auto& t:tests) {
        try { t.run(); std::cout << "PASS: " << t.name << '\n'; }
        catch (const std::exception& e) { ++failed; std::cerr << "FAIL: " << t.name << ": " << e.what() << '\n'; }
    }
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
