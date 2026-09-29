#include <csim/simulation/pendulum.hpp>
#include <csim/simulation/drone.hpp>
#include <csim/simulation/suspended_payload.hpp>

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

namespace {
void runSuspendedPayload() {
    using namespace csim::simulation;
    auto model = std::make_shared<SuspendedPayloadModel>(2, 0.5, 1.2,
        csim::dynamics::Drone::defaultInertia(), 9.81, 0.002);
    csim::dynamics::SuspendedPayloadState initial;
    initial.drone.position_W = {0,0,5};
    initial.cable_direction_W = csim::math::Vector3{0.3,0.2,-1}.normalized();
    initial.cable_angular_velocity_W = initial.cable_direction_W.cross({0.2,-0.3,0.1});
    const double thrust = 2.5*9.81;
    auto data = makeData(model, {thrust,{}}, initial);
    const double invariant = getState(*model,data).physical.energy-thrust*initial.drone.position_W.z;
    double max_work_error = 0;
    double min_tension = getState(*model,data).physical.tension;
    for (int i=0; i<5000; ++i) {
        step(*model,data);
        const auto s = getState(*model,data);
        max_work_error = std::max(max_work_error,
            std::abs(s.physical.energy-thrust*s.state.drone.position_W.z-invariant));
        min_tension = std::min(min_tension,s.physical.tension);
    }
    const auto s = getState(*model,data);
    std::cout << std::setprecision(10) << "Coupled drone and taut point payload: constant thrust, no damping\n"
              << "time: " << s.time << " s\n"
              << "payload_position_W: " << s.physical.payload_position_W.x << ", "
              << s.physical.payload_position_W.y << ", " << s.physical.payload_position_W.z << " m\n"
              << "minimum tension: " << min_tension << " N\n"
              << "max energy minus thrust work error: " << max_work_error << " J\n";
    if (max_work_error > 2e-8 || min_tension <= 0)
        throw std::runtime_error("Coupled example verification failed");
}

void runDrone() {
    using namespace csim::simulation;
    auto model = std::make_shared<DroneModel>(1, csim::dynamics::Drone::defaultInertia(), 9.80665, 0.005);
    auto data = makeData(model, {{0,0,5}, {}, {}, {}});
    const double hover_thrust = model->physics().mass() * model->physics().gravity();
    setControl(*model, data, {hover_thrust, {}});
    for (int i = 0; i < 200; ++i) step(*model, data);
    setControl(*model, data, {hover_thrust, {0,0,0.02}});
    for (int i = 0; i < 200; ++i) step(*model, data);
    const auto s = getState(*model, data);
    std::cout << std::setprecision(10) << "Unloaded drone: hover then yaw torque\n"
              << "time: " << s.time << " s\n"
              << "position_W: " << s.state.position_W.x << ", " << s.state.position_W.y
              << ", " << s.state.position_W.z << " m\n"
              << "angular_velocity_B: " << s.state.angular_velocity_B.x << ", "
              << s.state.angular_velocity_B.y << ", " << s.state.angular_velocity_B.z << " rad/s\n"
              << "q_WB (wxyz): " << s.state.q_WB.w << ", " << s.state.q_WB.x << ", "
              << s.state.q_WB.y << ", " << s.state.q_WB.z << '\n';
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "payload") {
            runSuspendedPayload();
            return EXIT_SUCCESS;
        }
        if (argc == 2 && std::string(argv[1]) == "drone") {
            runDrone();
            return EXIT_SUCCESS;
        }
        if (argc > 2 || (argc == 2 && std::string(argv[1]) != "pendulum")) {
            std::cerr << "Usage: simulator [pendulum|drone|payload]\n";
            return EXIT_FAILURE;
        }
        using namespace csim::simulation;
        auto model = std::make_shared<PendulumModel>(1, 1, 9.80665, 0.01);
        auto data = makeData(model, 0.7);
        const double initial_energy = getState(*model, data).physical.energy;
        double max_energy_error = 0;
        for (int i = 0; i < 2000; ++i) {
            step(*model, data);
            max_energy_error = std::max(max_energy_error,
                std::abs(getState(*model, data).physical.energy / initial_energy - 1));
        }
        const auto snapshot = getState(*model, data);
        std::cout << std::setprecision(10)
                  << "Passive planar pendulum\n"
                  << "time: " << snapshot.time << " s\n"
                  << "angle: " << snapshot.state.angle << " rad\n"
                  << "tension: " << snapshot.physical.tension << " N\n"
                  << "max relative energy error: " << max_energy_error << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Simulation failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
