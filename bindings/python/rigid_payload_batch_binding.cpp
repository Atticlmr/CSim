#include <csim/simulation/rigid_payload_batch.hpp>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <numeric>
#include <optional>
#include <cstdint>

namespace py = pybind11;
namespace {
using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;
using csim::simulation::RigidPayloadBatch;

void requireShape(const Array& array, std::size_t rows, py::ssize_t cols) {
    if (array.ndim() != 2 || array.shape(0) != static_cast<py::ssize_t>(rows)
        || array.shape(1) != cols)
        throw std::invalid_argument("Expected array shape (" + std::to_string(rows)
                                    + ", " + std::to_string(cols) + ")");
}

void writeBody(double* output, const csim::dynamics::DroneState& state) {
    output[0]=state.position_W.x; output[1]=state.position_W.y; output[2]=state.position_W.z;
    output[3]=state.velocity_W.x; output[4]=state.velocity_W.y; output[5]=state.velocity_W.z;
    output[6]=state.q_WB.w; output[7]=state.q_WB.x; output[8]=state.q_WB.y; output[9]=state.q_WB.z;
    output[10]=state.angular_velocity_B.x; output[11]=state.angular_velocity_B.y; output[12]=state.angular_velocity_B.z;
}

csim::dynamics::DroneState readBody(const double* input) {
    return {{input[0],input[1],input[2]}, {input[3],input[4],input[5]},
        {input[6],input[7],input[8],input[9]}, {input[10],input[11],input[12]}};
}

py::dict snapshot(RigidPayloadBatch& batch) {
    std::vector<csim::simulation::RigidPayloadBatchSnapshot> snapshots;
    {
        py::gil_scoped_release release;
        snapshots=batch.getState();
    }
    const auto count=static_cast<py::ssize_t>(snapshots.size());
    py::array_t<double> states({count,py::ssize_t{27}}), controls({count,py::ssize_t{4}}), times(count);
    py::array_t<std::int64_t> event_counts(count);
    py::array_t<double> event_losses(count), event_times(count);
    py::list event_types;
    auto state_values=states.mutable_unchecked<2>(), control_values=controls.mutable_unchecked<2>();
    auto time_values=times.mutable_unchecked<1>();
    auto count_values=event_counts.mutable_unchecked<1>();
    auto loss_values=event_losses.mutable_unchecked<1>(), event_time_values=event_times.mutable_unchecked<1>();
    for (py::ssize_t index=0;index<count;++index) {
        const auto& entry=snapshots[static_cast<std::size_t>(index)];
        double values[27];
        writeBody(values,entry.state.drone); writeBody(values+13,entry.state.payload); values[26]=entry.state.slack?1.:0.;
        for (py::ssize_t component=0;component<27;++component) state_values(index,component)=values[component];
        control_values(index,0)=entry.control.thrust; control_values(index,1)=entry.control.torque_B.x;
        control_values(index,2)=entry.control.torque_B.y; control_values(index,3)=entry.control.torque_B.z;
        time_values(index)=entry.time; count_values(index)=static_cast<std::int64_t>(entry.events.size());
        loss_values(index)=0; event_time_values(index)=entry.events.empty()?entry.time:entry.events.back().time;
        py::list types;
        for (const auto& event:entry.events) { types.append(event.type); loss_values(index)+=event.energy_loss; }
        event_types.append(types);
    }
    py::dict result;
    result["state"]=states; result["control"]=controls; result["time"]=times;
    result["event_count"]=event_counts; result["event_energy_loss"]=event_losses;
    result["event_time"]=event_times; result["event_type"]=event_types;
    return result;
}
}

void bindRigidPayloadBatch(py::module_& module) {
    py::class_<RigidPayloadBatch>(module,"RigidPayloadBatch")
        .def(py::init([](std::shared_ptr<csim::simulation::RigidPayloadModel> model,
                         std::size_t count,std::size_t threads) {
            py::gil_scoped_release release;
            return std::make_unique<RigidPayloadBatch>(std::move(model),count,threads);
        }),py::arg("model"),py::arg("num_envs"),py::arg("threads")=0)
        .def_property_readonly("model",&RigidPayloadBatch::model)
        .def_property_readonly("num_envs",&RigidPayloadBatch::size)
        .def_property_readonly("threads",&RigidPayloadBatch::threads)
        .def("step",[](RigidPayloadBatch& batch,const Array& actions,std::size_t substeps) {
            requireShape(actions,batch.size(),4);
            const auto values=actions.unchecked<2>();
            std::vector<csim::dynamics::DroneControl> controls(batch.size());
            for (std::size_t index=0;index<controls.size();++index)
                controls[index]={values(index,0),{values(index,1),values(index,2),values(index,3)}};
            py::gil_scoped_release release;
            batch.step(controls,substeps);
        },py::arg("actions"),py::arg("substeps")=1)
        .def("reset",[](RigidPayloadBatch& batch,const Array& states,
                         const std::optional<std::vector<std::size_t>>& indices) {
            std::vector<std::size_t> selected;
            if (indices) selected=*indices;
            else { selected.resize(batch.size()); std::iota(selected.begin(),selected.end(),0); }
            requireShape(states,selected.size(),27);
            const auto values=states.unchecked<2>();
            std::vector<csim::dynamics::RigidPayloadState> initial(selected.size());
            for (std::size_t index=0;index<initial.size();++index) {
                const double* row=&values(index,0);
                if (row[26]!=0 && row[26]!=1)
                    throw std::invalid_argument("Slack flag must be 0 or 1");
                initial[index]={readBody(row),readBody(row+13),row[26]!=0};
            }
            py::gil_scoped_release release;
            batch.reset(selected,initial);
        },py::arg("states"),py::arg("indices")=py::none())
        .def("get_state",[](RigidPayloadBatch& batch) { return snapshot(batch); });
}
