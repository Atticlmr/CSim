#include <csim/simulation/drone.hpp>

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
using namespace csim::simulation;
using csim::dynamics::Drone;
using csim::dynamics::DroneState;
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
void attitudeNear(Quaternion actual, Quaternion expected, double tolerance) {
    // The sign of a quaternion does not change the physical attitude.
    near(std::min((actual - expected).norm(), (actual + expected).norm()), 0, tolerance);
}
template <typename Error, typename F> void throws(F&& f) {
    try { f(); } catch (const Error&) { return; }
    throw std::runtime_error("expected exception missing");
}

void translation() {
    auto model = std::make_shared<DroneModel>(2, Drone::defaultInertia(), 9.81, 0.01);
    DroneState initial{{1,2,10}, {0.4,-0.3,0.5}, {}, {}};
    auto data = makeData(model, initial);
    for (int i = 0; i < 100; ++i) step(*model, data);
    auto s = getState(*model, data);
    near(s.time, 1);
    near(s.state.position_W, {1.4, 1.7, 10.5-4.905});
    near(s.state.velocity_W, {0.4,-0.3,0.5-9.81});
    attitudeNear(s.state.q_WB, {}, 0);
    near(s.physical.energy, model->physics().observe(initial, {}).energy, 1e-10);

    reset(*model, data, {{0,0,5}, {}, {}, {}});
    setControl(*model, data, {19.62, {}});
    for (int i = 0; i < 1000; ++i) step(*model, data);
    s = getState(*model, data);
    near(s.state.position_W, {0,0,5}, 0);
    near(s.state.velocity_W, {}, 0);
    near(s.physical.acceleration_W, {}, 0);

    const double pitch = std::acos(-1.0) / 6;
    reset(*model, data, {{}, {}, Quaternion::fromRollPitchYaw(0,pitch,0), {}});
    setControl(*model, data, {19.62/std::cos(pitch), {}});
    for (int i = 0; i < 100; ++i) step(*model, data);
    s = getState(*model, data);
    near(s.state.position_W, {0.5*9.81*std::tan(pitch),0,0});
    near(s.state.velocity_W, {9.81*std::tan(pitch),0,0});
}

void principalAxisTorque() {
    const Matrix3 inertia{0.02,0,0,0,0.03,0,0,0,0.04};
    auto model = std::make_shared<DroneModel>(1, inertia, 9.81, 0.005);
    const auto q0 = Quaternion::fromRollPitchYaw(0.4,-0.3,0.2);
    const Vector3 axes[]{{1,0,0},{0,1,0},{0,0,1}};
    for (int axis = 0; axis < 3; ++axis) {
        auto data = makeData(model, {{0,0,10}, {}, q0, axes[axis]*0.2});
        setControl(*model, data, {0, axes[axis]*(inertia(axis,axis)*0.6)});
        for (int i = 0; i < 200; ++i) step(*model, data);
        const auto s = getState(*model, data);
        near(s.state.angular_velocity_B, axes[axis]*0.8);
        attitudeNear(s.state.q_WB, q0*Quaternion::fromAxisAngle(axes[axis], 0.5), 1e-12);
        near(s.state.q_WB.norm(), 1, 1e-14);
    }
}

struct InvariantErrors { double energy = 0; double momentum = 0; };
InvariantErrors freeRotation(double h) {
    auto model = std::make_shared<DroneModel>(1, Matrix3{2,0,0,0,3,0,0,0,4}, 9.81, h);
    auto data = makeData(model, {{0,0,10}, {}, Quaternion::fromRollPitchYaw(0.2,0.3,-0.4), {1,2,3}});
    const auto initial = getState(*model, data);
    const double e0 = 25; // 0.5*(2*1^2+3*2^2+4*3^2).
    InvariantErrors error;
    const int steps = static_cast<int>(std::lround(10/h));
    for (int i = 0; i < steps; ++i) {
        step(*model, data);
        const auto s = getState(*model, data);
        const auto w = s.state.angular_velocity_B;
        const double energy = w.x*w.x + 1.5*w.y*w.y + 2*w.z*w.z;
        error.energy = std::max(error.energy, std::abs(energy/e0-1));
        error.momentum = std::max(error.momentum,
            (s.physical.angular_momentum_W-initial.physical.angular_momentum_W).norm()
                / initial.physical.angular_momentum_W.norm());
        near(s.state.q_WB.norm(), 1, 1e-14);
    }
    return error;
}
void conservedQuantities() {
    const auto coarse = freeRotation(0.04), medium = freeRotation(0.02), fine = freeRotation(0.01);
    std::cout << "Free rotation errors (h=0.04/0.02/0.01): E " << coarse.energy << '/' << medium.energy
              << '/' << fine.energy << ", H " << coarse.momentum << '/' << medium.momentum << '/' << fine.momentum << '\n';
    check(coarse.energy > 10*medium.energy && medium.energy > 10*fine.energy, "rotational energy convergence");
    check(coarse.momentum > 10*medium.momentum && medium.momentum > 10*fine.momentum, "world momentum convergence");
    check(fine.energy < 1e-7 && fine.momentum < 1e-7, "fine-step invariants");
}

void snapshotsControlsAndOwnership() {
    auto model = std::make_shared<DroneModel>();
    auto data = makeData(model, {{}, {}, {2,0,0,0}, {}});
    auto independent = makeData(model);
    near(getState(*model,data).state.q_WB.w, 1, 0);
    auto before = getState(*model, data);
    setControl(*model, data, {model->physics().mass()*model->physics().gravity(), {0,0,0.01}});
    const auto controlled = getState(*model, data);
    near(controlled.time, 0, 0);
    near(controlled.physical.acceleration_W, {});
    near(controlled.physical.angular_acceleration_B.z, 0.25);
    near(before.control.thrust, 0, 0);
    before.state.position_W.x = 99;
    before.control.thrust = 99;
    step(*model, data);
    near(getState(*model, independent).time, 0, 0);
    near(getState(*model, independent).control.thrust, 0, 0);
    check(getState(*model, data).control.thrust != 99, "snapshot must be independent");
    reset(*model, data, {{1,2,3}, {}, {-2,0,0,0}, {}});
    const auto reset_state = getState(*model,data);
    near(reset_state.time, 0, 0);
    near(reset_state.control.thrust, 0, 0);
    near(reset_state.control.torque_B, {}, 0);
    near(reset_state.state.position_W, {1,2,3}, 0);
    near(reset_state.state.q_WB.w, -1, 0);
    std::weak_ptr<DroneModel> weak = model;
    model.reset();
    check(!weak.expired(), "data must retain model");
    step(*data.model(), data);
}

void identityAndAtomicFailures() {
    auto model = std::make_shared<DroneModel>();
    auto other = std::make_shared<DroneModel>();
    auto data = makeData(model);
    throws<std::invalid_argument>([&]{ step(*other,data); });
    throws<std::invalid_argument>([&]{ getState(*other,data); });
    throws<std::invalid_argument>([&]{ reset(*other,data); });
    throws<std::invalid_argument>([&]{ setControl(*other,data,{}); });
    throws<std::invalid_argument>([]{ makeData(std::shared_ptr<DroneModel>{}); });
    setControl(*model, data, {10,{0,0,0.01}});
    throws<std::invalid_argument>([&]{ setControl(*model,data,{-1,{}}); });
    throws<std::overflow_error>([&]{ setControl(*model,data,{0,{0,0,1e308}}); });
    throws<std::domain_error>([&]{ reset(*model,data,{{},{},{0,0,0,0},{}}); });
    near(getState(*model,data).control.thrust, 10, 0);
    near(getState(*model,data).control.torque_B, {0,0,0.01}, 0);
    near(getState(*model,data).time, 0, 0);

    // Valid initial state/control, but intermediate RK arithmetic overflows.
    auto coarse = std::make_shared<DroneModel>(1, Drone::defaultInertia(), 9.81, 1e155);
    auto coarse_data = makeData(coarse, {{0,0,10},{},{},{}});
    setControl(*coarse, coarse_data, {11,{0.1,0.2,0.3}});
    const auto before = getState(*coarse,coarse_data);
    throws<std::overflow_error>([&]{ step(*coarse,coarse_data); });
    const auto after = getState(*coarse,coarse_data);
    near(after.time, before.time, 0);
    near(after.state.position_W, before.state.position_W, 0);
    near(after.state.velocity_W, before.state.velocity_W, 0);
    near(after.state.angular_velocity_B, before.state.angular_velocity_B, 0);
    attitudeNear(after.state.q_WB, before.state.q_WB, 0);
    near(after.control.thrust, before.control.thrust, 0);
    near(after.control.torque_B, before.control.torque_B, 0);
    auto tiny = std::make_shared<DroneModel>(1, Drone::defaultInertia(), 9.81,
                                           std::numeric_limits<double>::denorm_min());
    auto tiny_data = makeData(tiny);
    throws<std::overflow_error>([&]{ step(*tiny,tiny_data); });
    near(getState(*tiny,tiny_data).time, 0, 0);
    for (double h : {0.0,-1.0,std::numeric_limits<double>::infinity()})
        throws<std::invalid_argument>([&]{ DroneModel bad(1, Drone::defaultInertia(), 9.81, h); });
}
}

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]{{"free fall hover and inclined thrust",translation}, {"principal axis torque",principalAxisTorque},
        {"torque-free invariant convergence",conservedQuantities}, {"snapshots control reset and ownership",snapshotsControlsAndOwnership},
        {"model identity and atomic failures",identityAndAtomicFailures}};
    int failed = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& e) { ++failed; std::cerr << "FAIL: " << test.name << ": " << e.what() << '\n'; }
    }
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
