#include <csim/io/urdf.hpp>
#include <csim/io/mjcf.hpp>
#include <csim/simulation/drone.hpp>
#include <csim/simulation/suspended_payload.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <sstream>

namespace py=pybind11;
void bindModelLoading(py::module_& module) {
    module.def("load_model", [](const std::string& path, std::string format, bool free_base,
             const std::array<double,4>& q, const std::map<std::string,std::string>& packages,
             double gravity,double timestep,std::optional<double> payload_mass,double length, const csim::dynamics::DragConfig& drone_drag,
             const csim::dynamics::DragConfig& payload_drag,const csim::dynamics::WindField& wind, const std::string& integrator, double rtol, double atol, std::size_t max_substeps, const std::string& cable_mode) -> py::object {
        if (!payload_mass&&payload_drag.enabled()) throw std::invalid_argument("payload_drag requires payload_mass");
        if (format=="auto") format=std::filesystem::path(path).extension()==".urdf" ? "urdf" : "mjcf";
        csim::io::ImportOptions options;
        for (const auto& entry:packages) options.package_roots.emplace(entry.first,entry.second);
        csim::io::ImportResult imported;
        if (format=="urdf") imported=csim::io::loadUrdf(path,options);
        else if (format=="mjcf") imported=csim::io::loadMjcf(path,options);
        else throw std::invalid_argument("format must be auto, urdf or mjcf");
        if (!imported.description) {
            std::ostringstream message;
            for (const auto& d:imported.diagnostics) message<<d.source.file.string()<<":"<<d.source.line
                <<" "<<d.source.element<<": "<<d.message<<"\n";
            throw std::invalid_argument(message.str());
        }
        auto asset=std::make_shared<const csim::model::RigidBodyAsset>(
            csim::model::buildRigidBody(*imported.description,free_base,{q[0],q[1],q[2],q[3]}));
        for (const auto& body:imported.description->bodies) if (!body.collisions.empty()) {
            if (PyErr_WarnEx(PyExc_UserWarning,"URDF collision geometry was validated; CSim has no contact solver",1)<0)
                throw py::error_already_set();
            break;
        }
        if (payload_mass) return py::cast(std::make_shared<csim::simulation::SuspendedPayloadModel>(
            asset->mass,*payload_mass,length,asset->inertia_B,gravity,timestep,asset,drone_drag,payload_drag,wind,csim::simulation::IntegratorSettings{integrator,rtol,atol,max_substeps},cable_mode));
        return py::cast(std::make_shared<csim::simulation::DroneModel>(asset->mass,asset->inertia_B,gravity,timestep,asset,drone_drag,wind,csim::simulation::IntegratorSettings{integrator,rtol,atol,max_substeps}));
    },py::arg("path"),py::kw_only(),py::arg("format")="auto",py::arg("free_base")=false,
      py::arg("root_to_body")=std::array<double,4>{1,0,0,0},
      py::arg("package_roots")=std::map<std::string,std::string>{},py::arg("gravity")=9.80665,
      py::arg("timestep")=0.001,py::arg("payload_mass")=py::none(),py::arg("length")=1.0,
      py::arg("drone_drag")=csim::dynamics::DragConfig{},py::arg("payload_drag")=csim::dynamics::DragConfig{},
      py::arg("wind")=csim::dynamics::WindField{},py::arg("integrator")="rk4",py::arg("rtol")=1e-6,py::arg("atol")=1e-9,py::arg("max_substeps")=10000,py::arg("cable_mode")="taut",
      "Load a validated fixed assembly as one free drone; optional CoM-attached point payload. SI units, Z-up source world, root_to_body is wxyz R-to-FLU rotation.");
}
