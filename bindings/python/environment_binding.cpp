#include "environment_binding.hpp"
#include <csim/simulation/drone.hpp>
#include <csim/simulation/suspended_payload.hpp>
namespace py=pybind11;
using Triple=std::array<double,3>;
using Tensor=std::array<Triple,3>;
using csim::dynamics::DragConfig;
using csim::dynamics::WindField;
void bindEnvironment(py::module_& m) {
    py::class_<DragConfig>(m,"DragConfig")
        .def(py::init([](double k1,double k2,double k0,const std::string& mode,double epsilon) {
            DragConfig d{k1,k2,k0,mode,epsilon}; d.validate(); return d;
        }),py::kw_only(),py::arg("k1")=0,py::arg("k2")=0,py::arg("k0")=0,py::arg("sign_mode")="exact",py::arg("epsilon_v")=0)
        .def_readonly("k1",&DragConfig::k1).def_readonly("k2",&DragConfig::k2).def_readonly("k0",&DragConfig::k0)
        .def_readonly("sign_mode",&DragConfig::sign_mode).def_readonly("epsilon_v",&DragConfig::epsilon_v)
        .def("to_dict",&csim::binding::dragConfig);
    py::class_<WindField>(m,"WindField")
        .def(py::init([](const Triple& v,const Tensor& gradient,const Triple& origin,const Triple& gust,double frequency,double phase) {
            WindField w; w.velocity_W={v[0],v[1],v[2]}; w.reference_W={origin[0],origin[1],origin[2]};
            w.gust_amplitude_W={gust[0],gust[1],gust[2]}; w.gust_frequency=frequency; w.gust_phase=phase;
            for (std::size_t i=0;i<3;++i) for (std::size_t j=0;j<3;++j) w.gradient_W(i,j)=gradient[i][j];
            w.validate(); return w;
        }),py::kw_only(),py::arg("velocity_W")=Triple{},py::arg("gradient_W")=Tensor{},
            py::arg("reference_W")=Triple{},py::arg("gust_amplitude_W")=Triple{},
            py::arg("gust_frequency")=0,py::arg("gust_phase")=0)
        .def("to_dict",&csim::binding::windConfig);
}
template<class Model> py::dict config(const Model& m,const csim::dynamics::Drone& drone) {
    py::dict d; d["gravity"]=drone.gravity(); d["timestep"]=m.timestep();
    d["integrator"]=m.integration().method; d["rtol"]=m.integration().rtol;
    d["atol"]=m.integration().atol; d["max_substeps"]=m.integration().max_substeps;
    Tensor j{}; for (std::size_t a=0;a<3;++a) for (std::size_t b=0;b<3;++b) j[a][b]=drone.inertia()(a,b);
    d["inertia_B"]=j;
    d["drone_drag"]=csim::binding::dragConfig(drone.drag()); d["wind"]=csim::binding::windConfig(drone.wind());
    const auto pose=m.asset()?m.asset()->initial_pose_WB:csim::model::Pose{};
    py::dict p; p["position_W"]=csim::binding::triple(pose.position);
    p["q_WB"]=std::array<double,4>{pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z};
    d["initial_pose_WB"]=p;
    return d;
}
void bindModelConfig(py::module_& m) {
    m.def("get_config",[](const csim::simulation::DroneModel& model) {
        auto d=config(model,model.physics()); d["kind"]="drone"; d["mass"]=model.physics().mass(); return d;
    },py::arg("model"),"Copy all physical and wind parameters for reproducible experiments.");
    m.def("get_config",[](const csim::simulation::SuspendedPayloadModel& model) {
        auto d=config(model,model.physics().drone()); d["kind"]="suspended_payload";
        d["drone_mass"]=model.physics().drone().mass(); d["payload_mass"]=model.physics().payloadMass();
        d["length"]=model.physics().length(); d["cable_mode"]=model.cableMode(); d["payload_drag"]=csim::binding::dragConfig(model.physics().payloadDrag()); return d;
    },py::arg("model"));
}
