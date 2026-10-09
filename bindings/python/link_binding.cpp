#include "environment_binding.hpp"
#include <csim/simulation/link.hpp>
#include <pybind11/stl.h>

namespace py = pybind11;
namespace {
py::dict snapshot(const csim::simulation::LinkSnapshot& s) {
    using csim::binding::triple;
    py::dict result;
    result["name"] = s.name;
    result["parent"] = py::cast(s.parent);
    result["time"] = s.time;
    result["position_W"] = triple(s.pose_WL.position);
    const auto& q = s.pose_WL.orientation;
    result["q_WL"] = std::array<double,4>{q.w,q.x,q.y,q.z};
    result["velocity_W"] = triple(s.velocity_W);
    result["angular_velocity_W"] = triple(s.angular_velocity_W);
    result["acceleration_W"] = triple(s.acceleration_W);
    result["angular_acceleration_W"] = triple(s.angular_acceleration_W);
    result["com_position_W"] = triple(s.com_position_W);
    result["com_velocity_W"] = triple(s.com_velocity_W);
    result["com_acceleration_W"] = triple(s.com_acceleration_W);
    return result;
}

template<class Model, class Data> void bind(py::module_& module) {
    module.def("get_link_state", [](const Model& model, const Data& data, const std::string& name) {
        try { return snapshot(csim::simulation::getLinkState(model, data, name)); }
        catch (const std::out_of_range& error) { throw py::key_error(error.what()); }
    }, py::arg("model"), py::arg("data"), py::arg("name"),
       "Read one fixed source link by name. Origin and CoM quantities are in W; q_WL is wxyz. No time advance.");
    module.def("get_link_states", [](const Model& model, const Data& data) {
        py::dict result;
        for (const auto& s : csim::simulation::getLinkStates(model, data)) result[py::str(s.name)] = snapshot(s);
        return result;
    }, py::arg("model"), py::arg("data"),
       "Read independent snapshots of all fixed source links, keyed by name, from one physics observation.");
}
} // namespace

void bindLinkStates(py::module_& module) {
    bind<csim::simulation::DroneModel, csim::simulation::DroneData>(module);
    bind<csim::simulation::SuspendedPayloadModel, csim::simulation::SuspendedPayloadData>(module);
    bind<csim::simulation::RigidPayloadModel, csim::simulation::RigidPayloadData>(module);
}
