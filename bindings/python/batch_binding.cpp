#include <csim/simulation/drone_batch.hpp>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <numeric>
#include <optional>

namespace py = pybind11;
namespace {
using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;
using csim::simulation::DroneBatch;

void requireShape(const Array& array, std::size_t rows, py::ssize_t cols) {
    if (array.ndim() != 2 || array.shape(0) != static_cast<py::ssize_t>(rows)
        || array.shape(1) != cols) {
        throw std::invalid_argument("Expected array shape (" + std::to_string(rows)
                                    + ", " + std::to_string(cols) + ")");
    }
}

py::dict snapshot(DroneBatch& batch) {
    std::vector<csim::simulation::DroneBatchSnapshot> snapshots;
    {
        py::gil_scoped_release release;
        snapshots = batch.getState();
    }
    const auto count = static_cast<py::ssize_t>(snapshots.size());
    py::array_t<double> states({count, py::ssize_t{13}});
    py::array_t<double> controls({count, py::ssize_t{4}});
    py::array_t<double> times(count);
    auto state_values = states.mutable_unchecked<2>();
    auto control_values = controls.mutable_unchecked<2>();
    auto time_values = times.mutable_unchecked<1>();
    for (py::ssize_t index = 0; index < count; ++index) {
        const auto& entry = snapshots[index];
        const auto& state = entry.state;
        const double values[]{state.position_W.x, state.position_W.y, state.position_W.z,
            state.velocity_W.x, state.velocity_W.y, state.velocity_W.z,
            state.q_WB.w, state.q_WB.x, state.q_WB.y, state.q_WB.z,
            state.angular_velocity_B.x, state.angular_velocity_B.y, state.angular_velocity_B.z};
        for (py::ssize_t component = 0; component < 13; ++component) {
            state_values(index, component) = values[component];
        }
        control_values(index, 0) = entry.control.thrust;
        control_values(index, 1) = entry.control.torque_B.x;
        control_values(index, 2) = entry.control.torque_B.y;
        control_values(index, 3) = entry.control.torque_B.z;
        time_values(index) = entry.time;
    }
    py::dict result;
    result["state"] = states;
    result["control"] = controls;
    result["time"] = times;
    return result;
}
}

void bindDroneBatch(py::module_& module) {
    py::class_<DroneBatch>(module, "DroneBatch")
        .def(py::init([](std::shared_ptr<csim::simulation::DroneModel> model,
                         std::size_t count, std::size_t threads) {
            py::gil_scoped_release release;
            return std::make_unique<DroneBatch>(std::move(model), count, threads);
        }), py::arg("model"), py::arg("num_envs"), py::arg("threads") = 0)
        .def_property_readonly("model", &DroneBatch::model)
        .def_property_readonly("num_envs", &DroneBatch::size)
        .def_property_readonly("threads", &DroneBatch::threads)
        .def("step", [](DroneBatch& batch, const Array& actions, std::size_t substeps) {
            requireShape(actions, batch.size(), 4);
            const auto values = actions.unchecked<2>();
            std::vector<csim::dynamics::DroneControl> controls(batch.size());
            for (std::size_t index = 0; index < controls.size(); ++index) {
                controls[index] = {values(index, 0),
                    {values(index, 1), values(index, 2), values(index, 3)}};
            }
            py::gil_scoped_release release;
            batch.step(controls, substeps);
        }, py::arg("actions"), py::arg("substeps") = 1,
            "Advance all environments under actual thrust/torque; commit the entire batch on success.")
        .def("reset", [](DroneBatch& batch, const Array& states,
                          const std::optional<std::vector<std::size_t>>& indices) {
            std::vector<std::size_t> selected;
            if (indices) selected = *indices;
            else {
                selected.resize(batch.size());
                std::iota(selected.begin(), selected.end(), 0);
            }
            requireShape(states, selected.size(), 13);
            const auto values = states.unchecked<2>();
            std::vector<csim::dynamics::DroneState> initial(selected.size());
            for (std::size_t index = 0; index < initial.size(); ++index) {
                initial[index] = {{values(index, 0), values(index, 1), values(index, 2)},
                    {values(index, 3), values(index, 4), values(index, 5)},
                    {values(index, 6), values(index, 7), values(index, 8), values(index, 9)},
                    {values(index, 10), values(index, 11), values(index, 12)}};
            }
            py::gil_scoped_release release;
            batch.reset(selected, initial);
        }, py::arg("states"), py::arg("indices") = py::none(),
            "Reset selected environments, normalize attitudes and clear their times and controls.")
        .def("get_state", &snapshot,
            "Return owned NumPy arrays: state (N,13), control (N,4) and time (N,).");
}
