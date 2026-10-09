#include "window.hpp"
#include <csim/simulation/drone.hpp>
#include <csim/simulation/suspended_payload.hpp>
#include <csim/simulation/rigid_payload.hpp>
#include <pybind11/pybind11.h>
#include <variant>

namespace py=pybind11;
using namespace csim;
namespace {
class PythonWindow {
public:
    using Models=std::variant<std::shared_ptr<simulation::DroneModel>,std::shared_ptr<simulation::SuspendedPayloadModel>,
                             std::shared_ptr<simulation::RigidPayloadModel>>;
    PythonWindow(Models model,const std::string& shaders,int width,int height,bool hidden)
        : model_(std::move(model)), window_(shaders,width,height,hidden) {}
    template<class Model,class Data> void sync(const Data& data) {
        auto model=std::get_if<std::shared_ptr<Model>>(&model_);
        if (!model) throw std::invalid_argument("Data type does not match the viewer model");
        const auto s=simulation::getState(**model,data);
        viewer::Frame f;
        f.time=s.time;
        if constexpr (std::is_same_v<Model,simulation::DroneModel>) {
            f.drone_position_W=s.state.position_W; f.q_WB=s.state.q_WB; f.has_payload=false;
        } else if constexpr (std::is_same_v<Model,simulation::RigidPayloadModel>) {
            f.drone_position_W=s.state.drone.position_W; f.q_WB=s.state.drone.q_WB;
            f.payload_position_W=s.state.payload.position_W; f.q_WP=s.state.payload.q_WB;
            f.tension=s.physical.tension; f.cable_slack=s.state.slack; f.rigid_payload=true;
            f.drone_attachment_W=s.physical.drone_attachment.position;
            f.payload_attachment_W=s.physical.payload_attachment.position;
        } else {
            f.drone_position_W=s.state.drone.position_W; f.q_WB=s.state.drone.q_WB;
            f.payload_position_W=s.physical.payload_position_W; f.tension=s.physical.tension;
            f.cable_slack=s.physical.slack;
        }
        if constexpr (std::is_same_v<Model,simulation::RigidPayloadModel>)
            window_.sync(f,(*model)->asset().get(),(*model)->payloadAsset().get());
        else window_.sync(f,(*model)->asset().get());
    }
    Models model_;
    viewer::Window window_;
};
}
PYBIND11_MODULE(_csim_viewer,module) {
    py::module_::import("csim"); // Register the shared simulation model/data types first.
    py::class_<PythonWindow>(module,"Window")
        .def(py::init([](std::shared_ptr<simulation::DroneModel> m,const std::string& s,int w,int h,bool hidden) {
            if (!m) throw std::invalid_argument("Viewer model must not be null");
            return std::make_unique<PythonWindow>(m,s,w,h,hidden);
        }))
        .def(py::init([](std::shared_ptr<simulation::SuspendedPayloadModel> m,const std::string& s,int w,int h,bool hidden) {
            if (!m) throw std::invalid_argument("Viewer model must not be null");
            return std::make_unique<PythonWindow>(m,s,w,h,hidden);
        }))
        .def("sync",&PythonWindow::sync<simulation::DroneModel,simulation::DroneData>)
        .def(py::init([](std::shared_ptr<simulation::RigidPayloadModel> m,const std::string& s,int w,int h,bool hidden) {
            if (!m) throw std::invalid_argument("Viewer model must not be null");
            return std::make_unique<PythonWindow>(m,s,w,h,hidden);
        }))
        .def("sync",&PythonWindow::sync<simulation::RigidPayloadModel,simulation::RigidPayloadData>)
        .def("sync",&PythonWindow::sync<simulation::SuspendedPayloadModel,simulation::SuspendedPayloadData>)
        .def("is_running",[](const PythonWindow& w){ return w.window_.isRunning(); })
        .def("close",[](PythonWindow& w){ w.window_.close(); })
        .def("screenshot",[](PythonWindow& w,const std::string& path){ w.window_.screenshot(path); });
}
