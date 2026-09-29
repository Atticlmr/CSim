#include <csim/dynamics/suspended_payload.hpp>

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
using namespace csim::dynamics;
using csim::math::Vector3;
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void near(double a, double b, double tolerance = 1e-11) {
    check(std::isfinite(a) && std::abs(a-b) <= tolerance, "numeric mismatch");
}
void near(Vector3 a, Vector3 b, double tolerance = 1e-11) {
    near(a.x,b.x,tolerance); near(a.y,b.y,tolerance); near(a.z,b.z,tolerance);
}
template<class Error, class F> void throws(F&& f) {
    try { f(); } catch (const Error&) { return; }
    throw std::runtime_error("expected exception missing");
}

void hoverAndVerticalAcceleration() {
    const SuspendedPayload model(2, 0.5, 1.2, Drone::defaultInertia(), 9.81);
    SuspendedPayloadState state;
    state.drone.position_W = {1,2,5};
    const auto hover = model.observe(state, {2.5*9.81, {}});
    near(hover.drone.acceleration_W, {});
    near(hover.payload_acceleration_W, {});
    near(hover.tension, 0.5*9.81);
    near(hover.payload_position_W, {1,2,3.8});
    near(hover.cable_force_on_drone_W, {0,0,-0.5*9.81});
    near(hover.cable_force_on_payload_W, {0,0,0.5*9.81});
    near(hover.energy, 2*9.81*5 + 0.5*9.81*3.8);
    const auto rising = model.observe(state, {2.5*(9.81+3), {}});
    near(rising.drone.acceleration_W, {0,0,3});
    near(rising.payload_acceleration_W, {0,0,3});
    near(rising.tension, 0.5*(9.81+3));
    const auto d = model.derivative(state, {2.5*9.81, {}});
    near(d.drone.q_WB.norm(), 0, 0);
    near(d.cable_direction_W, {}); near(d.cable_angular_velocity_W, {});
}

void spatialBalancesAndPower() {
    const SuspendedPayload model(2, 0.5, 1.2);
    SuspendedPayloadState state;
    state.drone = {{1,2,3}, {0.4,-0.2,0.7},
        csim::math::Quaternion::fromRollPitchYaw(0.3,-0.2,0.4), {0.3,-0.4,0.2}};
    state.cable_direction_W = Vector3{0.3,0.4,-0.8}.normalized();
    state.cable_angular_velocity_W = state.cable_direction_W.cross(Vector3{0.6,-0.2,0.5});
    const DroneControl control{29, {0.1,-0.2,0.03}};
    const auto o = model.observe(state, control);
    const auto d = model.derivative(state, control);
    const Vector3 gravity{0,0,-model.drone().gravity()};
    near(2*(o.drone.acceleration_W-gravity), o.drone.thrust_W+o.cable_force_on_drone_W);
    near(0.5*(o.payload_acceleration_W-gravity), o.cable_force_on_payload_W);
    near(o.cable_force_on_drone_W+o.cable_force_on_payload_W, {});
    const auto ds = d.cable_direction_W;
    const auto dds = d.cable_angular_velocity_W.cross(state.cable_direction_W)
        + state.cable_angular_velocity_W.cross(ds);
    near(o.payload_acceleration_W-o.drone.acceleration_W, model.length()*dds);
    near(state.cable_direction_W.dot(ds), 0);
    near(state.cable_direction_W.dot(dds), -ds.squaredNorm());
    near(state.cable_direction_W.dot(d.cable_angular_velocity_W)
        + ds.dot(state.cable_angular_velocity_W), 0);
    const double cable_power = o.cable_force_on_drone_W.dot(state.drone.velocity_W)
        + o.cable_force_on_payload_W.dot(o.payload_velocity_W);
    near(cable_power, 0);
    constexpr double epsilon = 1e-6;
    const auto plus = SuspendedPayload::projectedState(state + d*epsilon);
    const auto minus = SuspendedPayload::projectedState(state + d*(-epsilon));
    near((model.observe(plus,control).energy-model.observe(minus,control).energy)/(2*epsilon),
        o.drone.thrust_W.dot(state.drone.velocity_W)+control.torque_B.dot(state.drone.angular_velocity_B), 3e-8);
}

void attachmentAndRotation() {
    const csim::math::Matrix3 inertia{2,0.2,0.1,0.2,3,0.3,0.1,0.3,4};
    const SuspendedPayload model(1,0.2,1,inertia);
    SuspendedPayloadState state;
    state.drone.angular_velocity_B = {1,2,3};
    state.drone.q_WB = csim::math::Quaternion::fromRollPitchYaw(0.2,0.3,0.4)*(-5);
    const DroneControl control{12,{5.125,-5.025,4.675}};
    const auto d = model.derivative(state,control);
    near(d.drone.angular_velocity_B, {0.5,-0.25,0.75});
    near(model.observe(state,control).drone.angular_acceleration_B, {0.5,-0.25,0.75});
    const auto qdot = model.drone().derivative(state.drone,control).q_WB;
    near((d.drone.q_WB-qdot).norm(), 0, 0);
    near(SuspendedPayload::normalizedState(state).drone.q_WB.norm(), 1);
}

void invalidParametersAndConstraints() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (double bad : {0.0,-1.0,nan,inf}) {
        throws<std::invalid_argument>([&]{ SuspendedPayload m(1,bad); });
        throws<std::invalid_argument>([&]{ SuspendedPayload m(1,1,bad); });
        throws<std::invalid_argument>([&]{ SuspendedPayload m(bad); });
    }
    throws<std::overflow_error>([]{ SuspendedPayload m(1e308,1e308); });
    SuspendedPayloadState state;
    state.cable_direction_W = {0,0,-2};
    throws<std::invalid_argument>([&]{ SuspendedPayload::normalizedState(state); });
    state.cable_direction_W = {};
    throws<std::invalid_argument>([&]{ SuspendedPayload::normalizedState(state); });
    state.cable_direction_W = {0,0,-1}; state.cable_angular_velocity_W = {0,1,0.1};
    throws<std::invalid_argument>([&]{ SuspendedPayload::normalizedState(state); });
    state.cable_angular_velocity_W = {nan,0,0};
    throws<std::invalid_argument>([&]{ SuspendedPayload::normalizedState(state); });
    state = {}; state.drone.position_W.z = 1e20;
    throws<std::overflow_error>([&]{ SuspendedPayload().observe(state,{12,{}}); });
    state = {}; state.cable_angular_velocity_W = {1e200,0,0};
    throws<std::overflow_error>([&]{ SuspendedPayload().observe(state,{12,{}}); });
    throws<std::invalid_argument>([]{ SuspendedPayload().observe({}, {-1,{}}); });
}

void tautBoundary() {
    const SuspendedPayload model;
    throws<CableDomainError>([&]{ model.observe({}, {}); });
    SuspendedPayloadState state;
    state.cable_direction_W = {0,0,1};
    throws<CableDomainError>([&]{ model.derivative(state, {12,{}}); });
    // Zero thrust can be valid when relative motion provides centripetal tension.
    state.cable_angular_velocity_W = {2,0,0};
    const auto orbit = model.observe(state, {});
    near(orbit.tension, (1.0/6)*4);
    near(orbit.cable_angular_acceleration_W, {});
}
}

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]{{"hover and vertical acceleration",hoverAndVerticalAcceleration},
        {"spatial force, constraint and power balances",spatialBalancesAndPower},
        {"centre attachment and full inertia",attachmentAndRotation},
        {"invalid parameters and constraints",invalidParametersAndConstraints},
        {"strict taut boundary",tautBoundary}};
    int failed = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& e) { ++failed; std::cerr << "FAIL: " << test.name << ": " << e.what() << '\n'; }
    }
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
