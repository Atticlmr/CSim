#include "environment_binding.hpp"
#include <csim/simulation/pendulum.hpp>
#include <csim/simulation/drone.hpp>
#include <csim/simulation/suspended_payload.hpp>
#include <csim/simulation/link.hpp>
#include <csim/simulation/wrench_step.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <memory>

namespace py = pybind11;
void bindModelLoading(py::module_&);
void bindLinkStates(py::module_&);
void bindDroneBatch(py::module_&);
void bindRigidPayload(py::module_&);
void bindRigidPayloadBatch(py::module_&);
using csim::simulation::PendulumModel;
using csim::simulation::PendulumData;
using csim::simulation::DroneModel;
using csim::simulation::DroneData;
using csim::simulation::SuspendedPayloadModel;
using csim::simulation::SuspendedPayloadData;

namespace {
std::array<double, 3> components(const csim::math::Vector3& value) {
    return {value.x, value.y, value.z};
}
using Triple = std::array<double, 3>;
using Attitude = std::array<double, 4>;
using Inertia = std::array<Triple, 3>;
csim::math::Vector3 vector(const Triple& v) { return {v[0], v[1], v[2]}; }
Inertia inertiaComponents(const csim::math::Matrix3& matrix) {
    Inertia result{};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) result[i][j] = matrix(i, j);
    return result;
}
csim::dynamics::DroneState droneState(const Triple& position, const Triple& velocity,
                                    const Attitude& q, const Triple& rate) {
    return {vector(position), vector(velocity), {q[0], q[1], q[2], q[3]}, vector(rate)};
}
template<class Model> csim::dynamics::DroneState initialState(const Model& model,
        const std::optional<Triple>& position, const Triple& velocity,
        const std::optional<Attitude>& q, const Triple& rate) {
    const auto pose=model.asset() ? model.asset()->initial_pose_WB : csim::model::Pose{};
    const auto orientation=pose.orientation;
    return droneState(position.value_or(components(pose.position)),velocity,
        q.value_or(Attitude{orientation.w,orientation.x,orientation.y,orientation.z}),rate);
}
}

PYBIND11_MODULE(csim, module) {
    module.doc() = "CSim physical simulation interface: pendulum, rigid-body drone and suspended payload.";
    bindEnvironment(module);
    bindRigidPayloadBatch(module);
    // Mathematical types, derivatives and integrators stay inside the C++ core.
    py::register_local_exception<csim::dynamics::PendulumDomainError>(
        module, "PendulumDomainError", PyExc_ValueError);
    py::register_local_exception<csim::dynamics::CableDomainError>(
        module, "CableDomainError", PyExc_ValueError);

    py::class_<PendulumModel, std::shared_ptr<PendulumModel>>(module, "PendulumModel")
        .def(py::init([](double length, double mass, double gravity, double timestep,
                         const std::array<double, 3>& pivot, const std::string& integrator, double rtol, double atol, std::size_t max_substeps) {
            return std::make_shared<PendulumModel>(length, mass, gravity, timestep,
                csim::math::Vector3{pivot[0], pivot[1], pivot[2]}, csim::simulation::IntegratorSettings{integrator,rtol,atol,max_substeps});
        }), py::arg("length") = 1.0, py::arg("mass") = 1.0, py::arg("gravity") = 9.80665,
            py::arg("timestep") = 0.001, py::arg("pivot_W") = std::array<double, 3>{0, 0, 0}, py::arg("integrator")="rk4",py::arg("rtol")=1e-6,py::arg("atol")=1e-9,py::arg("max_substeps")=10000)
        .def_property_readonly("length", [](const PendulumModel& m) { return m.physics().length(); })
        .def_property_readonly("mass", [](const PendulumModel& m) { return m.physics().mass(); })
        .def_property_readonly("gravity", [](const PendulumModel& m) { return m.physics().gravity(); })
        .def_property_readonly("timestep", &PendulumModel::timestep)
        .def_property_readonly("integrator", [](const PendulumModel& m) { return m.integration().method; })
        .def_property_readonly("rtol", [](const PendulumModel& m) { return m.integration().rtol; })
        .def_property_readonly("atol", [](const PendulumModel& m) { return m.integration().atol; })
        .def_property_readonly("max_substeps", [](const PendulumModel& m) { return m.integration().max_substeps; })
        .def_property_readonly("pivot_W", [](const PendulumModel& m) { return components(m.physics().pivot()); });

    py::class_<PendulumData>(module, "PendulumData")
        .def_property_readonly("model", &PendulumData::model);

    module.def("make_data", py::overload_cast<std::shared_ptr<PendulumModel>, double, double>(
                   &csim::simulation::makeData), py::arg("model"),
               py::arg("angle") = 0.0, py::arg("angular_velocity") = 0.0);
    module.def("step", py::overload_cast<const PendulumModel&, PendulumData&>(
                   &csim::simulation::step), py::arg("model"), py::arg("data"),
               "Advance one physical timestep in C++; commit state only on success.");
    module.def("reset", py::overload_cast<const PendulumModel&, PendulumData&, double, double>(
                   &csim::simulation::reset), py::arg("model"), py::arg("data"),
               py::arg("angle") = 0.0, py::arg("angular_velocity") = 0.0);
    module.def("get_state", [](const PendulumModel& model, const PendulumData& data) {
        const auto snapshot = csim::simulation::getState(model, data);
        py::dict result;
        result["time"] = snapshot.time;
        result["angle"] = snapshot.state.angle;
        result["angular_velocity"] = snapshot.state.angular_velocity;
        result["position_W"] = components(snapshot.physical.position_W);
        result["velocity_W"] = components(snapshot.physical.velocity_W);
        result["cable_direction_W"] = components(snapshot.physical.cable_direction_W);
        result["tension"] = snapshot.physical.tension;
        result["energy"] = snapshot.physical.energy;
        result["mode"] = "taut";
        return result;
    }, py::arg("model"), py::arg("data"), "Read an independent physical state snapshot without advancing time.");
    py::class_<DroneModel, std::shared_ptr<DroneModel>>(module, "DroneModel")
        .def(py::init([](double mass, const Inertia& inertia, double gravity, double timestep,
                         const csim::dynamics::DragConfig& drone_drag, const csim::dynamics::WindField& wind, const std::string& integrator, double rtol, double atol, std::size_t max_substeps) {
            csim::math::Matrix3 matrix;
            for (std::size_t i = 0; i < 3; ++i)
                for (std::size_t j = 0; j < 3; ++j) matrix(i, j) = inertia[i][j];
            return std::make_shared<DroneModel>(mass, matrix, gravity, timestep, nullptr, drone_drag, wind, csim::simulation::IntegratorSettings{integrator,rtol,atol,max_substeps});
        }), py::arg("mass") = 1.0,
            py::arg("inertia_B") = inertiaComponents(csim::dynamics::Drone::defaultInertia()),
            py::arg("gravity") = 9.80665, py::arg("timestep") = 0.001,
            py::arg("drone_drag")=csim::dynamics::DragConfig{},py::arg("wind")=csim::dynamics::WindField{},py::arg("integrator")="rk4",py::arg("rtol")=1e-6,py::arg("atol")=1e-9,py::arg("max_substeps")=10000)
        .def_property_readonly("mass", [](const DroneModel& m) { return m.physics().mass(); })
        .def_property_readonly("inertia_B", [](const DroneModel& m) { return inertiaComponents(m.physics().inertia()); })
        .def_property_readonly("gravity", [](const DroneModel& m) { return m.physics().gravity(); })
        .def_property_readonly("timestep", &DroneModel::timestep)
        .def_property_readonly("integrator", [](const DroneModel& m) { return m.integration().method; })
        .def_property_readonly("rtol", [](const DroneModel& m) { return m.integration().rtol; })
        .def_property_readonly("atol", [](const DroneModel& m) { return m.integration().atol; })
        .def_property_readonly("max_substeps", [](const DroneModel& m) { return m.integration().max_substeps; })
        .def_property_readonly("link_names", &csim::simulation::linkNames<DroneModel>);

    py::class_<DroneData>(module, "DroneData")
        .def_property_readonly("model", &DroneData::model);
    module.def("make_data", [](std::shared_ptr<DroneModel> model, const std::optional<Triple>& position,
                              const Triple& velocity, const std::optional<Attitude>& q, const Triple& rate) {
        if (!model) throw std::invalid_argument("Simulation model must not be null");
        return csim::simulation::makeData(model, initialState(*model,position,velocity,q,rate));
    }, py::arg("model"), py::arg("position_W") = py::none(), py::arg("velocity_W") = Triple{},
       py::arg("q_WB") = py::none(), py::arg("angular_velocity_B") = Triple{});
    module.def("step", py::overload_cast<const DroneModel&, DroneData&>(&csim::simulation::step),
               py::arg("model"), py::arg("data"), "Advance one physical timestep under the current actual thrust and body torque.");
    module.def("step", [](const DroneModel& model, DroneData& data, double thrust, const Triple& torque) {
        csim::simulation::stepWithControl(model,data,{thrust,vector(torque)});
    }, py::arg("model"), py::arg("data"), py::arg("thrust"), py::arg("torque_B")=Triple{});
    module.def("set_control", [](const DroneModel& model, DroneData& data, double thrust, const Triple& torque) {
        csim::simulation::setControl(model, data, {thrust, vector(torque)});
    }, py::arg("model"), py::arg("data"), py::arg("thrust"), py::arg("torque_B") = Triple{},
       "Replace total thrust (N, +Z_B) and centre-of-mass body torque (N m); no time advance.");
    module.def("reset", [](const DroneModel& model, DroneData& data, const std::optional<Triple>& position,
                          const Triple& velocity, const std::optional<Attitude>& q, const Triple& rate) {
        csim::simulation::reset(model, data, initialState(model,position,velocity,q,rate));
    }, py::arg("model"), py::arg("data"), py::arg("position_W") = py::none(),
       py::arg("velocity_W") = Triple{}, py::arg("q_WB") = py::none(),
       py::arg("angular_velocity_B") = Triple{}, "Reset time, state and control (zero thrust and torque).");
    module.def("get_state", [](const DroneModel& model, const DroneData& data) {
        const auto s = csim::simulation::getState(model, data);
        py::dict result;
        result["time"] = s.time;
        result["position_W"] = components(s.state.position_W);
        result["velocity_W"] = components(s.state.velocity_W);
        const auto& q = s.state.q_WB;
        result["q_WB"] = Attitude{q.w, q.x, q.y, q.z};
        result["angular_velocity_B"] = components(s.state.angular_velocity_B);
        result["acceleration_W"] = components(s.physical.acceleration_W);
        result["angular_acceleration_B"] = components(s.physical.angular_acceleration_B);
        result["thrust_W"] = components(s.physical.thrust_W);
        result["angular_momentum_W"] = components(s.physical.angular_momentum_W);
        result["energy"] = s.physical.energy;
        py::dict control;
        control["thrust"] = s.control.thrust;
        control["torque_B"] = components(s.control.torque_B);
        result["control"] = control;
        py::dict air; air["drone"]=csim::binding::airLoads(s.physical.aerodynamics);
        result["aerodynamics"]=air;
        return result;
    }, py::arg("model"), py::arg("data"), "Read an independent drone state snapshot at the current input.");
    py::class_<SuspendedPayloadModel, std::shared_ptr<SuspendedPayloadModel>>(module, "SuspendedPayloadModel")
        .def(py::init([](double drone_mass, double payload_mass, double length,
                         const Inertia& inertia, double gravity, double timestep,  const csim::dynamics::DragConfig& drone_drag,
                         const csim::dynamics::DragConfig& payload_drag,const csim::dynamics::WindField& wind, const std::string& integrator, double rtol, double atol, std::size_t max_substeps, const std::string& cable_mode, double event_max_step, double event_tolerance, std::size_t max_events) {
            csim::math::Matrix3 matrix;
            for (std::size_t i = 0; i < 3; ++i)
                for (std::size_t j = 0; j < 3; ++j) matrix(i,j) = inertia[i][j];
            return std::make_shared<SuspendedPayloadModel>(drone_mass, payload_mass, length, matrix, gravity, timestep, nullptr, drone_drag, payload_drag, wind, csim::simulation::IntegratorSettings{integrator,rtol,atol,max_substeps},cable_mode,csim::simulation::CableEventSettings{event_max_step,event_tolerance,max_events});
        }), py::arg("drone_mass") = 1.0, py::arg("payload_mass") = 0.2, py::arg("length") = 1.0,
            py::arg("inertia_B") = inertiaComponents(csim::dynamics::Drone::defaultInertia()),
            py::arg("gravity") = 9.80665, py::arg("timestep") = 0.001,
            py::arg("drone_drag")=csim::dynamics::DragConfig{},py::arg("payload_drag")=csim::dynamics::DragConfig{},
            py::arg("wind")=csim::dynamics::WindField{},py::arg("integrator")="rk4",py::arg("rtol")=1e-6,py::arg("atol")=1e-9,py::arg("max_substeps")=10000,py::arg("cable_mode")="taut",py::arg("event_max_step")=0.005,py::arg("event_tolerance")=1e-10,py::arg("max_events")=64)
        .def_property_readonly("drone_mass", [](const SuspendedPayloadModel& m) { return m.physics().drone().mass(); })
        .def_property_readonly("payload_mass", [](const SuspendedPayloadModel& m) { return m.physics().payloadMass(); })
        .def_property_readonly("length", [](const SuspendedPayloadModel& m) { return m.physics().length(); })
        .def_property_readonly("inertia_B", [](const SuspendedPayloadModel& m) { return inertiaComponents(m.physics().drone().inertia()); })
        .def_property_readonly("gravity", [](const SuspendedPayloadModel& m) { return m.physics().drone().gravity(); })
        .def_property_readonly("timestep", &SuspendedPayloadModel::timestep)
        .def_property_readonly("integrator", [](const SuspendedPayloadModel& m) { return m.integration().method; })
        .def_property_readonly("rtol", [](const SuspendedPayloadModel& m) { return m.integration().rtol; })
        .def_property_readonly("atol", [](const SuspendedPayloadModel& m) { return m.integration().atol; })
        .def_property_readonly("max_substeps", [](const SuspendedPayloadModel& m) { return m.integration().max_substeps; })
        .def_property_readonly("event_max_step", [](const SuspendedPayloadModel& m) { return m.events().max_step; })
        .def_property_readonly("event_tolerance", [](const SuspendedPayloadModel& m) { return m.events().time_tolerance; })
        .def_property_readonly("max_events", [](const SuspendedPayloadModel& m) { return m.events().max_events; })
        .def_property_readonly("cable_mode", &SuspendedPayloadModel::cableMode)
        .def_property_readonly("link_names", &csim::simulation::linkNames<SuspendedPayloadModel>);
    py::class_<SuspendedPayloadData>(module, "SuspendedPayloadData")
        .def_property_readonly("model", &SuspendedPayloadData::model);
    module.def("make_data", [](std::shared_ptr<SuspendedPayloadModel> model, double thrust,
                               const Triple& torque, const std::optional<Triple>& position, const Triple& velocity,
                               const std::optional<Attitude>& q, const Triple& rate, const Triple& direction, const Triple& cable_rate,
                               const std::optional<Triple>& payload_position, const Triple& payload_velocity, const std::string& cable_mode) {
        if (!model) throw std::invalid_argument("Simulation model must not be null");
        if (cable_mode != "taut" && cable_mode != "slack") throw std::invalid_argument("cable_mode must be taut or slack");
        csim::dynamics::SuspendedPayloadState state{initialState(*model,position,velocity,q,rate),vector(direction),vector(cable_rate)};
        state.slack=(cable_mode=="slack");
        if (state.slack) {
            if (!payload_position) throw std::invalid_argument("Slack state requires payload_position_W");
            state.payload_position_W=vector(*payload_position); state.payload_velocity_W=vector(payload_velocity);
        }
        return csim::simulation::makeData(model, {thrust,vector(torque)}, state);
    }, py::arg("model"), py::arg("thrust"), py::arg("torque_B") = Triple{},
       py::arg("position_W") = py::none(), py::arg("velocity_W") = Triple{},
       py::arg("q_WB") = py::none(), py::arg("angular_velocity_B") = Triple{},
       py::arg("cable_direction_W") = Triple{0,0,-1}, py::arg("cable_angular_velocity_W") = Triple{},
       py::arg("payload_position_W") = py::none(), py::arg("payload_velocity_W") = Triple{}, py::arg("cable_mode") = "taut",
       "Create a taut state, or a slack state with cable_mode=hybrid and explicit payload position.");
    module.def("step", py::overload_cast<const SuspendedPayloadModel&, SuspendedPayloadData&>(&csim::simulation::step),
               py::arg("model"), py::arg("data"), "Advance one physical timestep, locating hybrid cable events internally; commit only on success.");
    module.def("step", [](const SuspendedPayloadModel& model, SuspendedPayloadData& data, double thrust, const Triple& torque) {
        csim::simulation::stepWithControl(model,data,{thrust,vector(torque)});
    }, py::arg("model"), py::arg("data"), py::arg("thrust"), py::arg("torque_B")=Triple{});
    module.def("set_control", [](const SuspendedPayloadModel& model, SuspendedPayloadData& data,
                                 double thrust, const Triple& torque) {
        csim::simulation::setControl(model,data,{thrust,vector(torque)});
    }, py::arg("model"), py::arg("data"), py::arg("thrust"), py::arg("torque_B") = Triple{});
    module.def("reset", [](const SuspendedPayloadModel& model, SuspendedPayloadData& data, double thrust,
                           const Triple& torque, const std::optional<Triple>& position, const Triple& velocity,
                           const std::optional<Attitude>& q, const Triple& rate, const Triple& direction, const Triple& cable_rate,
                           const std::optional<Triple>& payload_position, const Triple& payload_velocity, const std::string& cable_mode) {
        if (cable_mode != "taut" && cable_mode != "slack") throw std::invalid_argument("cable_mode must be taut or slack");
        csim::dynamics::SuspendedPayloadState state{initialState(model,position,velocity,q,rate),vector(direction),vector(cable_rate)};
        state.slack=(cable_mode=="slack");
        if (state.slack) { if (!payload_position) throw std::invalid_argument("Slack state requires payload_position_W"); state.payload_position_W=vector(*payload_position); state.payload_velocity_W=vector(payload_velocity); }
        csim::simulation::reset(model,data,{thrust,vector(torque)},state);
    }, py::arg("model"), py::arg("data"), py::arg("thrust"), py::arg("torque_B") = Triple{},
       py::arg("position_W") = py::none(), py::arg("velocity_W") = Triple{}, py::arg("q_WB") = py::none(),
       py::arg("angular_velocity_B") = Triple{}, py::arg("cable_direction_W") = Triple{0,0,-1}, py::arg("cable_angular_velocity_W") = Triple{},
       py::arg("payload_position_W") = py::none(), py::arg("payload_velocity_W") = Triple{}, py::arg("cable_mode") = "taut",
       "Reset physical state, optionally entering the hybrid slack mode.");
    module.def("get_state", [](const SuspendedPayloadModel& model, const SuspendedPayloadData& data) {
        const auto s = csim::simulation::getState(model,data);
        py::dict result;
        result["time"] = s.time;
        result["position_W"] = components(s.state.drone.position_W);
        result["velocity_W"] = components(s.state.drone.velocity_W);
        const auto& q = s.state.drone.q_WB;
        result["q_WB"] = Attitude{q.w,q.x,q.y,q.z};
        result["angular_velocity_B"] = components(s.state.drone.angular_velocity_B);
        result["acceleration_W"] = components(s.physical.drone.acceleration_W);
        result["angular_acceleration_B"] = components(s.physical.drone.angular_acceleration_B);
        result["thrust_W"] = components(s.physical.drone.thrust_W);
        result["angular_momentum_W"] = components(s.physical.drone.angular_momentum_W);
        result["cable_direction_W"] = components(s.state.cable_direction_W);
        result["cable_angular_velocity_W"] = components(s.state.cable_angular_velocity_W);
        result["cable_angular_acceleration_W"] = components(s.physical.cable_angular_acceleration_W);
        result["payload_position_W"] = components(s.physical.payload_position_W);
        result["payload_velocity_W"] = components(s.physical.payload_velocity_W);
        result["payload_acceleration_W"] = components(s.physical.payload_acceleration_W);
        result["cable_force_on_drone_W"] = components(s.physical.cable_force_on_drone_W);
        result["cable_force_on_payload_W"] = components(s.physical.cable_force_on_payload_W);
        result["tension"] = s.physical.tension;
        result["energy"] = s.physical.energy;
        result["mode"] = s.physical.slack ? "slack" : "taut";
        result["cable_distance"] = s.physical.cable_distance;
        result["cable_radial_velocity"] = s.physical.cable_radial_velocity;
        result["cable_impulse_W"] = components(s.physical.cable_impulse_W);
        py::list events;
        double loss=0;
        for (const auto& e:s.cable_events) {
            py::dict event;
            event["type"]=e.type; event["time"]=e.time;
            event["impulse_W"]=components(e.impulse_W); event["energy_loss"]=e.energy_loss;
            event["radial_velocity_before"]=e.radial_velocity_before;
            event["radial_velocity_after"]=e.radial_velocity_after;
            event["mode_after"]=e.slack_after ? "slack" : "taut";
            events.append(event); loss+=e.energy_loss;
        }
        result["cable_events"]=events;
        result["impact_energy_loss"]=loss;
        py::dict control;
        control["thrust"] = s.control.thrust;
        control["torque_B"] = components(s.control.torque_B);
        result["control"] = control;
        py::dict air; air["drone"]=csim::binding::airLoads(s.physical.drone.aerodynamics);
        air["payload"]=csim::binding::airLoads(s.physical.payload_aerodynamics); result["aerodynamics"]=air;
        return result;
    }, py::arg("model"), py::arg("data"), "Read both bodies, cable geometry and forces as an independent snapshot.");
    bindModelLoading(module);
    bindRigidPayload(module);
    bindLinkStates(module);
    bindModelConfig(module);
    bindDroneBatch(module);

}
