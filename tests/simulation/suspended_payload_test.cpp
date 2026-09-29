#include <csim/simulation/suspended_payload.hpp>

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
using namespace csim::simulation;
using csim::dynamics::SuspendedPayloadState;
using csim::dynamics::CableDomainError;
using csim::dynamics::Drone;
using csim::math::Vector3;
void check(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void near(double a, double b, double tolerance = 1e-10) {
    check(std::isfinite(a) && std::abs(a-b) <= tolerance, "numeric mismatch");
}
void near(Vector3 a, Vector3 b, double tolerance = 1e-10) {
    near(a.x,b.x,tolerance); near(a.y,b.y,tolerance); near(a.z,b.z,tolerance);
}
template<class Error, class F> void throws(F&& f) {
    try { f(); } catch (const Error&) { return; }
    throw std::runtime_error("expected exception missing");
}
auto modelWithStep(double dt) {
    return std::make_shared<SuspendedPayloadModel>(2,0.5,1.2,Drone::defaultInertia(),9.81,dt);
}
void unchanged(const SuspendedPayloadSnapshot& a, const SuspendedPayloadSnapshot& b) {
    near(a.time,b.time,0); near(a.state.drone.position_W,b.state.drone.position_W,0);
    near(a.state.drone.velocity_W,b.state.drone.velocity_W,0);
    near((a.state.drone.q_WB-b.state.drone.q_WB).norm(),0,0);
    near(a.state.drone.angular_velocity_B,b.state.drone.angular_velocity_B,0);
    near(a.state.cable_direction_W,b.state.cable_direction_W,0);
    near(a.state.cable_angular_velocity_W,b.state.cable_angular_velocity_W,0);
    near(a.control.thrust,b.control.thrust,0); near(a.control.torque_B,b.control.torque_B,0);
    near(a.physical.energy,b.physical.energy,0); near(a.physical.tension,b.physical.tension,0);
}

void hoverAndCollectiveThrust() {
    const auto model = modelWithStep(0.005);
    SuspendedPayloadState state;
    state.drone.position_W = {1,2,5};
    auto data = makeData(model,{2.5*9.81,{}},state);
    for (int i=0; i<2000; ++i) step(*model,data);
    auto result = getState(*model,data);
    near(result.time,10); near(result.state.drone.position_W,{1,2,5});
    near(result.physical.payload_position_W,{1,2,3.8});
    near(result.physical.tension,0.5*9.81);
    reset(*model,data,{2.5*(9.81+3),{}},state);
    for (int i=0; i<200; ++i) step(*model,data);
    result = getState(*model,data);
    near(result.state.drone.position_W,{1,2,6.5});
    near(result.state.drone.velocity_W,{0,0,3});
    near(result.physical.payload_position_W,{1,2,5.3});
    near(result.physical.payload_velocity_W,{0,0,3});
}

double freeOrbitError(double dt) {
    const auto model = modelWithStep(dt);
    const Vector3 axis = Vector3{1,2,3}.normalized();
    const Vector3 initial_direction = axis.cross({0,0,1}).normalized();
    const Vector3 omega = axis*2;
    const Vector3 initial_com{0.3,-0.4,50}, initial_com_velocity{0.2,-0.1,0.5};
    const double offset = 0.5/2.5*1.2;
    SuspendedPayloadState state;
    state.cable_direction_W = initial_direction;
    state.cable_angular_velocity_W = omega;
    state.drone.position_W = initial_com-initial_direction*offset;
    state.drone.velocity_W = initial_com_velocity-omega.cross(initial_direction)*offset;
    auto data = makeData(model,{},state);
    const double energy = getState(*model,data).physical.energy;
    double max_error = 0, max_energy_error = 0;
    for (int i=0; i<static_cast<int>(std::lround(2/dt)); ++i) {
        step(*model,data);
        const auto s = getState(*model,data);
        const double t = (i+1)*dt;
        const auto direction = initial_direction*std::cos(2*t)+axis.cross(initial_direction)*std::sin(2*t);
        const auto com = initial_com+initial_com_velocity*t+Vector3{0,0,-0.5*9.81*t*t};
        const auto velocity = initial_com_velocity+Vector3{0,0,-9.81*t};
        max_error = std::max({max_error,(s.state.cable_direction_W-direction).norm(),
            (s.state.drone.position_W-(com-direction*offset)).norm(),
            (s.state.drone.velocity_W-(velocity-omega.cross(direction)*offset)).norm(),
            (s.physical.payload_position_W-(com+direction*(1.2-offset))).norm()});
        max_energy_error = std::max(max_energy_error,std::abs(s.physical.energy-energy));
        near(s.state.cable_direction_W.norm(),1,5e-16);
        near(s.state.cable_direction_W.dot(s.state.cable_angular_velocity_W),0,1e-15);
        near(s.physical.tension,0.4*1.2*4,1e-12);
    }
    std::cout << "orbit dt=" << dt << " max state error=" << max_error
              << " max energy error=" << max_energy_error << '\n';
    check(max_energy_error < 0.02,"free orbit energy error too large");
    return max_error;
}
void orbitConvergence() {
    const double coarse = freeOrbitError(0.04);
    const double medium = freeOrbitError(0.02);
    const double fine = freeOrbitError(0.01);
    check(coarse/medium>14 && medium/fine>14,"coupled RK4 must converge at fourth order");
    check(fine<1e-7,"analytic free orbit mismatch");
}

void poweredSpatialSwing() {
    const auto model = modelWithStep(0.002);
    SuspendedPayloadState state;
    state.drone.position_W = {0,0,5};
    state.cable_direction_W = Vector3{0.3,0.2,-1}.normalized();
    state.cable_angular_velocity_W = state.cable_direction_W.cross({0.2,-0.3,0.1});
    const double thrust = 2.5*9.81;
    auto data = makeData(model,{thrust,{}},state);
    const auto start = getState(*model,data);
    const double invariant = start.physical.energy-thrust*start.state.drone.position_W.z;
    const auto momentum = state.drone.velocity_W*2+start.physical.payload_velocity_W*0.5;
    double max_energy_error=0;
    for (int i=0; i<5000; ++i) {
        step(*model,data);
        const auto s = getState(*model,data);
        const auto delta = s.physical.payload_position_W-s.state.drone.position_W;
        const auto relative_v = s.physical.payload_velocity_W-s.state.drone.velocity_W;
        near(delta.norm(),1.2,2e-15); near(delta.dot(relative_v),0,2e-15);
        near(s.state.drone.velocity_W*2+s.physical.payload_velocity_W*0.5,momentum,2e-8);
        max_energy_error=std::max(max_energy_error,
            std::abs(s.physical.energy-thrust*s.state.drone.position_W.z-invariant));
        check(s.physical.tension>0,"positive tension lost");
    }
    std::cout << "10 s spatial swing max (energy - thrust*z) error=" << max_energy_error << '\n';
    check(max_energy_error<2e-8,"external thrust work balance failed");
}

void controlsOwnershipAndReset() {
    auto model = modelWithStep(0.01);
    auto data = makeData(model,{24.525,{}});
    auto independent = makeData(model,{24.525,{}});
    auto old = getState(*model,data);
    setControl(*model,data,{30,{0,0,0.02}});
    const auto current = getState(*model,data);
    near(current.time,0,0); near(current.physical.drone.acceleration_W,{0,0,2.19});
    near(current.physical.drone.angular_acceleration_B,{0,0,0.5});
    old.state.drone.position_W.x=99;
    near(old.state.drone.position_W.x,99,0); near(old.control.thrust,24.525,0);
    near(getState(*model,data).state.drone.position_W.x,0,0);
    for (int i=0; i<100; ++i) step(*model,data);
    const auto yaw = getState(*model,data);
    near(yaw.state.drone.angular_velocity_B,{0,0,0.5});
    near((yaw.state.drone.q_WB-csim::math::Quaternion{std::cos(0.125),0,0,std::sin(0.125)}).norm(),0,1e-11);
    near(getState(*model,independent).time,0,0);
    SuspendedPayloadState reset_state;
    reset_state.drone.position_W = {1,2,5}; reset_state.drone.q_WB = {-2,0,0,0};
    reset(*model,data,{24.525,{}},reset_state);
    const auto reset_snapshot = getState(*model,data);
    near(reset_snapshot.time,0,0); near(reset_snapshot.control.torque_B,{},0);
    near(reset_snapshot.control.thrust,24.525,0); near(reset_snapshot.state.drone.q_WB.w,-1,0);
    model.reset(); step(*data.model(),data);
    near(getState(*data.model(),data).time,0.01);
}

void atomicFailures() {
    auto model = modelWithStep(0.01);
    auto data = makeData(model,{24.525,{}});
    step(*model,data);
    const auto before = getState(*model,data);
    auto other = modelWithStep(0.01);
    throws<std::invalid_argument>([&]{ step(*other,data); });
    throws<std::invalid_argument>([&]{ getState(*other,data); });
    throws<std::invalid_argument>([&]{ setControl(*other,data,{24.525,{}}); });
    throws<std::invalid_argument>([&]{ reset(*other,data,{24.525,{}}); });
    throws<std::invalid_argument>([]{ makeData(std::shared_ptr<SuspendedPayloadModel>{},{12,{}}); });
    throws<CableDomainError>([&]{ setControl(*model,data,{}); });
    throws<CableDomainError>([&]{ reset(*model,data,{}); });
    throws<std::overflow_error>([&]{ setControl(*model,data,{24.525,{1e308,0,0}}); });
    SuspendedPayloadState invalid; invalid.cable_direction_W={1,1,1};
    throws<std::invalid_argument>([&]{ reset(*model,data,{24.525,{}},invalid); });
    unchanged(getState(*model,data),before);
    // Valid initial tension, but a coarse step reaches the non-taut branch.
    auto coarse = modelWithStep(0.2);
    SuspendedPayloadState swinging;
    swinging.cable_direction_W={std::sin(1.55),0,-std::cos(1.55)};
    swinging.cable_angular_velocity_W={0,-1,0};
    auto boundary = makeData(coarse,{24.525,{}},swinging);
    const auto initial = getState(*coarse,boundary);
    throws<CableDomainError>([&]{ step(*coarse,boundary); });
    unchanged(getState(*coarse,boundary),initial);
    for (double dt : {1e155,std::numeric_limits<double>::denorm_min()}) {
        auto extreme = modelWithStep(dt);
        auto extreme_data = makeData(extreme,{30,{}});
        const auto snapshot = getState(*extreme,extreme_data);
        throws<std::overflow_error>([&]{ step(*extreme,extreme_data); });
        unchanged(getState(*extreme,extreme_data),snapshot);
    }
    for (double bad : {0.0,-1.0,std::numeric_limits<double>::infinity()})
        throws<std::invalid_argument>([&]{ modelWithStep(bad); });
}
}

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]{{"hover and collective thrust",hoverAndCollectiveThrust},
        {"analytic free orbit and fourth order convergence",orbitConvergence},
        {"spatial swing constraints and external work",poweredSpatialSwing},
        {"controls, ownership and reset",controlsOwnershipAndReset},
        {"atomic failures and tension boundary",atomicFailures}};
    int failed=0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& e) { ++failed; std::cerr << "FAIL: " << test.name << ": " << e.what() << '\n'; }
    }
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
