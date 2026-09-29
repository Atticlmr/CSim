#include <csim/dynamics/drone.hpp>

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
using namespace csim::dynamics;
using csim::math::Vector3;
using csim::math::Matrix3;
using csim::math::Quaternion;
void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance = 1e-12) {
    check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, "numeric mismatch");
}
void near(Vector3 actual, Vector3 expected, double tolerance = 1e-12) {
    near(actual.x, expected.x, tolerance); near(actual.y, expected.y, tolerance); near(actual.z, expected.z, tolerance);
}
template <typename Error, typename F> void throws(F&& f) {
    try { f(); } catch (const Error&) { return; }
    throw std::runtime_error("expected exception missing");
}

void restAndGravity() {
    const Drone drone(2, Drone::defaultInertia(), 9.81);
    DroneState state;
    const auto free = drone.derivative(state, {});
    near(free.velocity_W, {0, 0, -9.81});
    near(free.position_W, {});
    near(free.angular_velocity_B, {});
    near(free.q_WB.norm(), 0, 0); // Derivative is zero, never identity quaternion.
    const auto hover = drone.observe(state, {19.62, {}});
    near(hover.acceleration_W, {});
    near(hover.thrust_W, {0, 0, 19.62});
    near(hover.energy, 0);
}

void tiltedThrust() {
    const Drone drone(2, Drone::defaultInertia(), 9.81);
    DroneState state;
    const double pi = std::acos(-1.0);
    state.q_WB = Quaternion::fromRollPitchYaw(0, pi / 6, pi / 2);
    const auto physical = drone.observe(state, {20, {}});
    near(physical.thrust_W, {0, 10, 10 * std::sqrt(3.0)});
    near(physical.acceleration_W, {0, 5, 5 * std::sqrt(3.0) - 9.81});
    state.q_WB = Quaternion::fromRollPitchYaw(pi / 6, 0, 0);
    near(drone.observe(state, {20, {}}).thrust_W, {0, -10, 10 * std::sqrt(3.0)});
}

void eulerEquation() {
    const Drone diagonal(1, Matrix3{2, 0, 0, 0, 3, 0, 0, 0, 4});
    DroneState state;
    state.angular_velocity_B = {1, 2, 3};
    near(diagonal.derivative(state, {}).angular_velocity_B, {-3, 2, -0.5});
    // Off-diagonal inertia with independently evaluated J*Omega and cross product.
    const Drone general(1, Matrix3{2, 0.2, 0.1, 0.2, 3, 0.3, 0.1, 0.3, 4});
    near(general.derivative(state, {0, {5.125, -5.025, 4.675}}).angular_velocity_B,
         {0.5, -0.25, 0.75});
    near(general.observe(state, {}).angular_momentum_W, {2.7, 7.1, 12.7});
}

void attitudeConvention() {
    const Drone drone;
    DroneState state;
    state.q_WB = Quaternion::fromRollPitchYaw(0.3, -0.4, 0.7);
    state.angular_velocity_B = {0.8, -0.2, 0.6};
    const auto rate = drone.derivative(state, {});
    constexpr double epsilon = 1e-6;
    const auto forward = (state.q_WB + rate.q_WB * epsilon).toRotationMatrix();
    const auto backward = (state.q_WB + rate.q_WB * -epsilon).toRotationMatrix();
    const auto expected = state.q_WB.toRotationMatrix() * csim::math::skew(state.angular_velocity_B);
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) near((forward(i,j)-backward(i,j))/(2*epsilon), expected(i,j), 2e-10);
    const auto original = drone.observe(state, {3, {}});
    state.q_WB = state.q_WB * -7;
    near(drone.observe(state, {3, {}}).thrust_W, original.thrust_W);
    near(Drone::normalizedState(state).q_WB.norm(), 1);
}

void energyAndPower() {
    const Drone drone(2, Matrix3{2, 0.2, 0.1, 0.2, 3, 0.3, 0.1, 0.3, 4}, 9.81);
    const DroneState state{{1, 2, 3}, {0.4, -0.2, 0.7},
        Quaternion::fromRollPitchYaw(0.4, -0.2, 0.1), {1, 2, 3}};
    const DroneControl input{25, {0.3, -0.1, 0.2}};
    const auto physical = drone.observe(state, input);
    near(physical.energy, 0.69 + 27.5 + 58.86);
    const auto rate = drone.derivative(state, input);
    constexpr double epsilon = 1e-6;
    const double power = (drone.observe(state + rate * epsilon, input).energy
                         - drone.observe(state + rate * -epsilon, input).energy) / (2 * epsilon);
    near(power, physical.thrust_W.dot(state.velocity_W) + input.torque_B.dot(state.angular_velocity_B), 2e-8);
}

void invalidInputs() {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (double bad : {0.0, -1.0, inf, nan}) {
        throws<std::invalid_argument>([&]{ Drone d(bad); });
        throws<std::invalid_argument>([&]{ Drone d(1, Drone::defaultInertia(), bad); });
    }
    for (const auto& bad : {Matrix3{}, Matrix3{1,0,0,0,-1,0,0,0,-1},
            Matrix3{1,2,0,2,1,0,0,0,1}, Matrix3{1,0.1,0,0,1,0,0,0,1},
            Matrix3{1,0,0,0,1,0,0,0,nan}}) {
        throws<std::invalid_argument>([&]{ Drone d(1, bad); });
    }
    throws<csim::math::SingularMatrixError>([]{ Drone d(1, Matrix3{1,0,0,0,1,0,0,0,1e-18}); });
    auto rounded = Drone::defaultInertia();
    rounded(0,1) = 1e-16;
    const Drone canonical(1, rounded);
    near(canonical.inertia()(0,1), canonical.inertia()(1,0), 0);
    for (double bad : {-1.0, inf, nan})
        throws<std::invalid_argument>([&]{ Drone().derivative({}, {bad, {}}); });
    throws<std::invalid_argument>([&]{ Drone().derivative({}, {0, {0,nan,0}}); });
    DroneState state;
    state.q_WB = {0,0,0,0};
    throws<std::domain_error>([&]{ Drone::normalizedState(state); });
    state.q_WB = {};
    state.position_W.x = nan;
    throws<std::invalid_argument>([&]{ Drone::normalizedState(state); });
    state = {};
    state.angular_velocity_B = {1e200, 2e200, 3e200};
    throws<std::overflow_error>([&]{ Drone().derivative(state, {}); });
    throws<std::overflow_error>([]{ Drone(1e-300).derivative({}, {1e300, {}}); });
}
}

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]{{"rest and gravity", restAndGravity}, {"tilted thrust in W", tiltedThrust},
        {"full inertia and gyroscopic coupling", eulerEquation}, {"body attitude convention", attitudeConvention},
        {"energy and input power", energyAndPower}, {"invalid inputs", invalidInputs}};
    int failed = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& e) { ++failed; std::cerr << "FAIL: " << test.name << ": " << e.what() << '\n'; }
    }
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
