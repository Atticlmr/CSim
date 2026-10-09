#include <csim/simulation/drone_batch.hpp>

#include <array>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
using namespace csim::simulation;
using csim::dynamics::DroneControl;
using csim::dynamics::DroneState;

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

template<class Error, class Function> void throws(Function&& function) {
    try { function(); } catch (const Error&) { return; }
    throw std::runtime_error("Expected exception missing");
}

std::array<double, 18> values(const DroneBatchSnapshot& snapshot) {
    const auto& state = snapshot.state;
    const auto& control = snapshot.control;
    return {snapshot.time, state.position_W.x, state.position_W.y, state.position_W.z,
        state.velocity_W.x, state.velocity_W.y, state.velocity_W.z,
        state.q_WB.w, state.q_WB.x, state.q_WB.y, state.q_WB.z,
        state.angular_velocity_B.x, state.angular_velocity_B.y, state.angular_velocity_B.z,
        control.thrust, control.torque_B.x, control.torque_B.y, control.torque_B.z};
}

void equal(const std::vector<DroneBatchSnapshot>& actual,
           const std::vector<DroneBatchSnapshot>& expected) {
    check(actual.size() == expected.size(), "Snapshot count mismatch");
    for (std::size_t index = 0; index < actual.size(); ++index) {
        check(values(actual[index]) == values(expected[index]), "Batch snapshot mismatch");
    }
}

void scalarParity() {
    for (const auto& method : {"euler", "midpoint", "heun", "rk4", "dopri5"}) {
        auto model = std::make_shared<DroneModel>(1, csim::math::Matrix3{2,.1,0,.1,3,.2,0,.2,4},
            9.81, .002, nullptr, csim::dynamics::DragConfig{}, csim::dynamics::WindField{},
            IntegratorSettings{method});
        for (std::size_t threads : {1u, 3u, 20u, 0u}) {
            DroneBatch batch(model, 7, threads);
            check(batch.threads() >= 1 && batch.threads() <= 7, "Thread count bounded by batch");
            std::vector<std::size_t> indices;
            std::vector<DroneState> states;
            std::vector<DroneData> reference;
            std::vector<DroneControl> controls;
            for (std::size_t index = 0; index < 7; ++index) {
                const double offset = static_cast<double>(index);
                DroneState state{{offset,.1,10}, {.2,-.1,.3},
                    csim::math::Quaternion::fromRollPitchYaw(.1,.2,offset*.1), {.1,.2,.3}};
                indices.push_back(index);
                states.push_back(state);
                reference.push_back(makeData(model, state));
                controls.push_back({9.81 + offset*.1, {.02,-.01,offset*.001}});
            }
            batch.reset(indices, states);
            for (std::size_t iteration = 0; iteration < 12; ++iteration) {
                controls[0].thrust += .01;
                batch.step(controls, 3);
                std::vector<DroneBatchSnapshot> expected;
                for (std::size_t index = 0; index < reference.size(); ++index) {
                    setControl(*model, reference[index], controls[index]);
                    for (int substep = 0; substep < 3; ++substep) step(*model, reference[index]);
                    const auto snapshot = getState(*model, reference[index]);
                    expected.push_back({snapshot.time, snapshot.state, snapshot.control});
                }
                equal(batch.getState(), expected);
            }
        }
    }
}

void resetAndFailures() {
    auto model = std::make_shared<DroneModel>();
    throws<std::invalid_argument>([&] { DroneBatch invalid(nullptr, 1); });
    throws<std::invalid_argument>([&] { DroneBatch invalid(model, 0); });
    DroneBatch batch(model, 7, 3);
    std::vector<DroneControl> controls(7, {10, {0,0,.01}});
    batch.step(controls, 2);
    const auto before = batch.getState();
    throws<std::invalid_argument>([&] { batch.step({}, 1); });
    throws<std::invalid_argument>([&] { batch.step(controls, 0); });
    controls[6].thrust = -1;
    throws<std::invalid_argument>([&] { batch.step(controls); });
    controls[6].thrust = std::numeric_limits<double>::quiet_NaN();
    throws<std::invalid_argument>([&] { batch.step(controls); });
    throws<std::invalid_argument>([&] { batch.reset({1}, {}); });
    throws<std::out_of_range>([&] { batch.reset({7}, {DroneState{}}); });
    throws<std::invalid_argument>([&] { batch.reset({1,1}, {DroneState{}, DroneState{}}); });
    DroneState invalid;
    invalid.q_WB = {0,0,0,0};
    throws<std::domain_error>([&] { batch.reset({1,4}, {DroneState{}, invalid}); });
    equal(batch.getState(), before);
    batch.reset({}, {});
    equal(batch.getState(), before);
    DroneState initial{{1,2,3}, {}, {2,0,0,0}, {}};
    batch.reset({4,1}, {initial, DroneState{}});
    const auto after = batch.getState();
    check(after[4].time == 0 && after[4].control.thrust == 0, "Reset clears time and control");
    check(after[4].state.q_WB.w == 1 && after[4].state.position_W.z == 3, "Reset normalizes state");
    for (std::size_t index : {0u,2u,3u,5u,6u}) {
        check(values(after[index]) == values(before[index]), "Reset changed unselected environment");
    }
    auto copied = batch.getState();
    copied[0].state.position_W.x = 99;
    equal(batch.getState(), after);

    auto coarse = std::make_shared<DroneModel>(1, csim::dynamics::Drone::defaultInertia(), 9.81, 1e155);
    DroneBatch failing(coarse, 7, 3);
    const auto old = failing.getState();
    controls.assign(7, {9.81, {}});
    controls[6] = {11, {.1,.2,.3}};
    throws<std::overflow_error>([&] { failing.step(controls); });
    equal(failing.getState(), old);
    controls[6] = {9.81, {}};
    failing.step(controls);
    for (const auto& snapshot : failing.getState()) {
        check(snapshot.time == 1e155 && snapshot.state.position_W.z == 0, "Pool failed to recover");
    }
}

void concurrentCallsAndLifetime() {
    auto model = std::make_shared<DroneModel>();
    std::weak_ptr<DroneModel> weak = model;
    {
        DroneBatch batch(model, 17, 4);
        model.reset();
        check(!weak.expired(), "Batch must retain its model");
        const std::vector<DroneControl> controls(17, {9.80665, {}});
        std::atomic<bool> failed{false};
        auto advance = [&] {
            try {
                for (int iteration = 0; iteration < 30; ++iteration) {
                    batch.step(controls, 2);
                    const auto snapshots = batch.getState();
                    for (const auto& snapshot : snapshots) {
                        check(snapshot.time == snapshots[0].time, "Snapshot read a partial batch");
                    }
                }
            } catch (...) { failed = true; }
        };
        std::thread first(advance);
        std::thread second(advance);
        first.join();
        second.join();
        check(!failed, "Concurrent batch calls failed");
        double expected_time = 0;
        for (int iteration = 0; iteration < 120; ++iteration) expected_time += .001;
        for (const auto& snapshot : batch.getState()) {
            check(snapshot.time == expected_time, "Concurrent calls lost a step");
        }
    }
    check(weak.expired(), "Batch leaked its model");
    for (int iteration = 0; iteration < 20; ++iteration) {
        DroneBatch idle(std::make_shared<DroneModel>(), 1, 8);
        check(idle.threads() == 1, "Singleton batch should not create extra workers");
        DroneBatch pool(std::make_shared<DroneModel>(), 3, 3);
    }
}
}

int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]{{"single environment parity for all integrators", scalarParity},
        {"selective reset and atomic failures", resetAndFailures},
        {"concurrent calls and worker lifetime", concurrentCallsAndLifetime}};
    int failed = 0;
    for (const auto& test : tests) {
        try { test.run(); std::cout << "PASS: " << test.name << '\n'; }
        catch (const std::exception& error) {
            ++failed;
            std::cerr << "FAIL: " << test.name << ": " << error.what() << '\n';
        }
    }
    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
