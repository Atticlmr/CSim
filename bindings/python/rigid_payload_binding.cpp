#include "environment_binding.hpp"
#include <csim/io/urdf.hpp>
#include <csim/io/mjcf.hpp>
#include <csim/simulation/rigid_payload.hpp>
#include <csim/simulation/link.hpp>
#include <csim/simulation/wrench_step.hpp>
#include <fstream>
#include <sstream>

namespace py=pybind11;
namespace {
using csim::math::Vector3;
using csim::math::Quaternion;
using csim::binding::triple;
using csim::simulation::RigidPayloadModel;
using csim::simulation::RigidPayloadData;
using Triple=std::array<double,3>;
using Attitude=std::array<double,4>;
using Inertia=std::array<Triple,3>;

Vector3 vector(const Triple& value) { return {value[0],value[1],value[2]}; }
Quaternion quaternion(const Attitude& value) { return {value[0],value[1],value[2],value[3]}; }
Attitude attitude(const Quaternion& value) { return {value.w,value.x,value.y,value.z}; }
Inertia inertia(const csim::math::Matrix3& value) {
    Inertia result{};
    for (std::size_t row=0;row<3;++row)
        for (std::size_t column=0;column<3;++column) result[row][column]=value(row,column);
    return result;
}
void keys(const py::dict& object,std::initializer_list<const char*> allowed) {
    for (const auto& entry:object) {
        const auto name=py::cast<std::string>(entry.first);
        bool known=false;
        for (const auto* key:allowed) if (name==key) known=true;
        if (!known) throw std::invalid_argument("Unknown configuration field: "+name);
    }
}
py::dict readJson(const std::string& path) {
    std::error_code error;
    const auto size=std::filesystem::file_size(path,error);
    if (error || size>4*1024*1024) throw std::invalid_argument("Cannot read JSON or file exceeds 4 MiB: "+path);
    std::ifstream stream(path);
    if (!stream) throw std::invalid_argument("Cannot read JSON: "+path);
    std::ostringstream content; content<<stream.rdbuf();
    auto pairs=py::cpp_function([](const py::list& entries) {
        py::dict result;
        for (const auto& item:entries) {
            auto entry=py::cast<py::tuple>(item);
            if (result.contains(entry[0])) throw std::invalid_argument("Duplicate JSON key: "+py::cast<std::string>(entry[0]));
            result[entry[0]]=entry[1];
        }
        return result;
    });
    auto constant=py::cpp_function([](const std::string&) -> py::object {
        throw std::invalid_argument("JSON requires finite numbers");
    });
    auto result=py::module_::import("json").attr("loads")(content.str(),
        py::arg("object_pairs_hook")=pairs,py::arg("parse_constant")=constant);
    if (!py::isinstance<py::dict>(result)) throw std::invalid_argument("JSON root must be an object: "+path);
    return result.cast<py::dict>();
}
double number(py::handle value) {
    if (py::isinstance<py::bool_>(value) || (!py::isinstance<py::float_>(value) && !py::isinstance<py::int_>(value)))
        throw std::invalid_argument("Expected a JSON number");
    const auto result=py::cast<double>(value);
    if (!std::isfinite(result)) throw std::invalid_argument("JSON requires finite numbers");
    return result;
}
template<std::size_t count> std::array<double,count> numbers(py::handle value) {
    if (!py::isinstance<py::list>(value)) throw std::invalid_argument("Expected a JSON array");
    const auto sequence=py::reinterpret_borrow<py::list>(value);
    if (sequence.size()!=count) throw std::invalid_argument("Wrong JSON array length");
    std::array<double,count> result{};
    for (std::size_t index=0;index<count;++index) result[index]=number(sequence[index]);
    return result;
}
py::handle required(const py::dict& object,const char* name) {
    if (!object.contains(name)) throw std::invalid_argument(std::string("Missing configuration field: ")+name);
    return object[name];
}
csim::math::Matrix3 tensor(py::handle value) {
    if (!py::isinstance<py::list>(value) || py::len(value)!=3)
        throw std::invalid_argument("Expected a 3 by 3 array");
    const auto rows=py::reinterpret_borrow<py::list>(value);
    csim::math::Matrix3 result;
    for (std::size_t row=0;row<3;++row) {
        const auto values=numbers<3>(rows[row]);
        for (std::size_t column=0;column<3;++column) result(row,column)=values[column];
    }
    return result;
}
std::size_t countField(const py::dict& config,const char* name) {
    const double value=number(required(config,name));
    if (value<1 || value>1000000 || std::floor(value)!=value)
        throw std::invalid_argument(std::string("Invalid ")+name);
    return static_cast<std::size_t>(value);
}
csim::dynamics::DragConfig dragConfig(py::handle value) {
    const auto config=py::cast<py::dict>(value);
    keys(config,{"k1","k2","k0","sign_mode","epsilon_v"});
    return {number(required(config,"k1")),number(required(config,"k2")),number(required(config,"k0")),
        py::cast<std::string>(required(config,"sign_mode")),number(required(config,"epsilon_v"))};
}
std::shared_ptr<RigidPayloadModel> modelFromConfig(const py::dict& config) {
    keys(config,{"kind","drone_mass","payload_mass","inertia_B","payload_inertia_P",
        "drone_attachment_B","payload_attachment_P","length","gravity","timestep","integrator",
        "rtol","atol","max_substeps","cable_mode","initial_direction_W","initial_pose_WB",
        "initial_payload_q_WP","event_max_step","event_tolerance","max_events","drone_drag","payload_drag","wind"});
    if (py::cast<std::string>(required(config,"kind"))!="rigid_payload")
        throw std::invalid_argument("Expected rigid_payload configuration");
    auto drone=std::make_shared<csim::model::RigidBodyAsset>();
    auto payload=std::make_shared<csim::model::RigidBodyAsset>();
    drone->mass=number(required(config,"drone_mass"));
    payload->mass=number(required(config,"payload_mass"));
    drone->inertia_B=tensor(required(config,"inertia_B"));
    payload->inertia_B=tensor(required(config,"payload_inertia_P"));
    csim::model::validateInertial({drone->mass,{},drone->inertia_B,{}});
    csim::model::validateInertial({payload->mass,{},payload->inertia_B,{}});
    drone->attachments_B["cable_attachment"]={vector(numbers<3>(required(config,"drone_attachment_B"))),{}};
    payload->attachments_B["cable_attachment"]={vector(numbers<3>(required(config,"payload_attachment_P"))),{}};
    const auto pose=py::cast<py::dict>(required(config,"initial_pose_WB"));
    keys(pose,{"position_W","q_WB"});
    drone->initial_pose_WB={vector(numbers<3>(required(pose,"position_W"))),quaternion(numbers<4>(required(pose,"q_WB")))};
    payload->initial_pose_WB.orientation=quaternion(numbers<4>(required(config,"initial_payload_q_WP")));
    (void)csim::dynamics::Drone::normalizedState({drone->initial_pose_WB.position,{},drone->initial_pose_WB.orientation,{}});
    (void)payload->initial_pose_WB.orientation.normalized();
    csim::simulation::RigidCableSettings cable;
    cable.length=number(required(config,"length"));
    cable.mode=py::cast<std::string>(required(config,"cable_mode"));
    cable.initial_direction_W=vector(numbers<3>(required(config,"initial_direction_W")));
    cable.events={number(required(config,"event_max_step")),number(required(config,"event_tolerance")),countField(config,"max_events")};
    const auto wind_config=py::cast<py::dict>(required(config,"wind"));
    keys(wind_config,{"velocity_W","gradient_W","reference_W","gust_amplitude_W","gust_frequency","gust_phase"});
    csim::dynamics::WindField wind;
    wind.velocity_W=vector(numbers<3>(required(wind_config,"velocity_W")));
    wind.gradient_W=tensor(required(wind_config,"gradient_W"));
    wind.reference_W=vector(numbers<3>(required(wind_config,"reference_W")));
    wind.gust_amplitude_W=vector(numbers<3>(required(wind_config,"gust_amplitude_W")));
    wind.gust_frequency=number(required(wind_config,"gust_frequency"));
    wind.gust_phase=number(required(wind_config,"gust_phase"));
    return std::make_shared<RigidPayloadModel>(drone,payload,cable,number(required(config,"gravity")),
        number(required(config,"timestep")),csim::simulation::IntegratorSettings{
            py::cast<std::string>(required(config,"integrator")),number(required(config,"rtol")),
            number(required(config,"atol")),countField(config,"max_substeps")},
        dragConfig(required(config,"drone_drag")),dragConfig(required(config,"payload_drag")),wind);
}
csim::model::ModelDescription jsonBody(const std::string& path) {
    const auto config=readJson(path);
    keys(config,{"name","mass","inertia","center_of_mass","cable_attachment","initial_pose"});
    csim::model::ModelDescription description;
    description.name=py::cast<std::string>(required(config,"name"));
    description.root_motion=csim::model::RootMotion::free;
    csim::model::BodyDescription root;
    root.name="base";
    csim::model::InertialDescription physical;
    physical.mass=number(required(config,"mass"));
    if (config.contains("center_of_mass")) physical.body_from_inertial.position=vector(numbers<3>(config["center_of_mass"]));
    auto matrix=required(config,"inertia");
    if (!py::isinstance<py::list>(matrix) || py::len(matrix)!=3) throw std::invalid_argument("inertia must be a 3 by 3 array");
    const auto rows=py::reinterpret_borrow<py::list>(matrix);
    for (std::size_t row=0;row<3;++row) {
        const auto values=numbers<3>(rows[row]);
        for (std::size_t column=0;column<3;++column) physical.inertia(row,column)=values[column];
    }
    root.inertial=physical;
    if (config.contains("initial_pose")) {
        if (!py::isinstance<py::dict>(config["initial_pose"])) throw std::invalid_argument("initial_pose must be an object");
        const auto pose=py::cast<py::dict>(config["initial_pose"]);
        keys(pose,{"position_W","q_WR"});
        if (pose.contains("position_W")) root.parent_from_body.position=vector(numbers<3>(pose["position_W"]));
        if (pose.contains("q_WR")) root.parent_from_body.orientation=quaternion(numbers<4>(pose["q_WR"]));
    }
    csim::model::BodyDescription marker;
    marker.name="cable_attachment"; marker.parent=root.name;
    marker.parent_from_body.position=vector(numbers<3>(required(config,"cable_attachment")));
    description.bodies={root,marker};
    return description;
}
std::shared_ptr<const csim::model::RigidBodyAsset> loadBody(const std::string& path,const Attitude& rotation,
        const std::map<std::string,std::string>& packages) {
    const auto suffix=std::filesystem::path(path).extension().string();
    csim::model::ModelDescription description;
    if (suffix==".json") description=jsonBody(path);
    else {
        csim::io::ImportOptions options;
        for (const auto& entry:packages) options.package_roots.emplace(entry.first,entry.second);
        csim::io::ImportResult imported;
        if (suffix==".urdf") imported=csim::io::loadUrdf(path,options);
        else if (suffix==".xml" || suffix==".mjcf") imported=csim::io::loadMjcf(path,options);
        else throw std::invalid_argument("Body file must be .json, .urdf, .xml or .mjcf");
        if (!imported.description) {
            std::ostringstream message;
            for (const auto& diagnostic:imported.diagnostics)
                message<<diagnostic.source.file.string()<<":"<<diagnostic.source.line<<" "<<diagnostic.message<<"\n";
            throw std::invalid_argument(message.str());
        }
        description=std::move(*imported.description);
    }
    auto result=std::make_shared<const csim::model::RigidBodyAsset>(
        csim::model::buildRigidBody(description,true,quaternion(rotation)));
    (void)csim::simulation::cableAttachment(*result);
    for (const auto& body:description.bodies) if (!body.collisions.empty()) {
        if (PyErr_WarnEx(PyExc_UserWarning,"Collision geometry was validated; CSim has no contact solver",1)<0)
            throw py::error_already_set();
        break;
    }
    return result;
}
csim::simulation::RigidCableSettings loadCable(const std::string& path) {
    const auto config=readJson(path);
    keys(config,{"length","mode","initial_direction_W","event_max_step","event_tolerance","max_events"});
    csim::simulation::RigidCableSettings result;
    result.length=number(required(config,"length"));
    if (config.contains("mode")) result.mode=py::cast<std::string>(config["mode"]);
    if (config.contains("initial_direction_W")) result.initial_direction_W=vector(numbers<3>(config["initial_direction_W"]));
    if (config.contains("event_max_step")) result.events.max_step=number(config["event_max_step"]);
    if (config.contains("event_tolerance")) result.events.time_tolerance=number(config["event_tolerance"]);
    if (config.contains("max_events")) {
        const double value=number(config["max_events"]);
        if (value<1 || value>100000 || std::floor(value)!=value) throw std::invalid_argument("Invalid max_events");
        result.events.max_events=static_cast<std::size_t>(value);
    }
    result.validate();
    return result;
}
csim::dynamics::RigidPayloadState initialState(const RigidPayloadModel& model,const py::kwargs& fields) {
    keys(fields,{"position_W","velocity_W","q_WB","angular_velocity_B","payload_position_W",
        "payload_velocity_W","payload_q_WP","payload_angular_velocity_P","mode"});
    auto state=model.initialState();
    if (fields.contains("position_W")) state.drone.position_W=vector(py::cast<Triple>(fields["position_W"]));
    if (fields.contains("velocity_W")) state.drone.velocity_W=vector(py::cast<Triple>(fields["velocity_W"]));
    if (fields.contains("q_WB")) state.drone.q_WB=quaternion(py::cast<Attitude>(fields["q_WB"]));
    if (fields.contains("angular_velocity_B")) state.drone.angular_velocity_B=vector(py::cast<Triple>(fields["angular_velocity_B"]));
    if (fields.contains("payload_q_WP")) state.payload.q_WB=quaternion(py::cast<Attitude>(fields["payload_q_WP"]));
    if (fields.contains("payload_angular_velocity_P")) state.payload.angular_velocity_B=vector(py::cast<Triple>(fields["payload_angular_velocity_P"]));
    state.drone=csim::dynamics::Drone::normalizedState(state.drone);
    state.payload=csim::dynamics::Drone::normalizedState(state.payload);
    const auto anchor=csim::dynamics::RigidPayload::attachment(state.drone,model.physics().droneAttachment());
    state.payload.position_W=anchor.position+model.cable().initial_direction_W.normalized()*model.physics().length()
        -state.payload.q_WB.rotate(model.physics().payloadAttachment());
    state.payload.velocity_W=anchor.velocity-state.payload.q_WB.rotate(
        state.payload.angular_velocity_B.cross(model.physics().payloadAttachment()));
    if (fields.contains("payload_position_W")) state.payload.position_W=vector(py::cast<Triple>(fields["payload_position_W"]));
    if (fields.contains("payload_velocity_W")) state.payload.velocity_W=vector(py::cast<Triple>(fields["payload_velocity_W"]));
    if (fields.contains("mode")) {
        const auto mode=py::cast<std::string>(fields["mode"]);
        if (mode!="taut" && mode!="slack") throw std::invalid_argument("Initial mode must be taut or slack");
        state.slack=mode=="slack";
    }
    return state;
}
py::dict snapshot(const csim::simulation::RigidPayloadSnapshot& value) {
    const auto& state=value.state;
    const auto& physical=value.physical;
    py::dict result;
    result["time"]=value.time;
    result["position_W"]=triple(state.drone.position_W);
    result["velocity_W"]=triple(state.drone.velocity_W);
    result["q_WB"]=attitude(state.drone.q_WB);
    result["angular_velocity_B"]=triple(state.drone.angular_velocity_B);
    result["acceleration_W"]=triple(physical.drone.acceleration_W);
    result["angular_acceleration_B"]=triple(physical.drone.angular_acceleration_B);
    result["payload_position_W"]=triple(state.payload.position_W);
    result["payload_velocity_W"]=triple(state.payload.velocity_W);
    result["payload_q_WP"]=attitude(state.payload.q_WB);
    result["payload_angular_velocity_P"]=triple(state.payload.angular_velocity_B);
    result["payload_acceleration_W"]=triple(physical.payload.acceleration_W);
    result["payload_angular_acceleration_P"]=triple(physical.payload.angular_acceleration_B);
    result["drone_attachment_position_W"]=triple(physical.drone_attachment.position);
    result["payload_attachment_position_W"]=triple(physical.payload_attachment.position);
    result["drone_attachment_velocity_W"]=triple(physical.drone_attachment.velocity);
    result["payload_attachment_velocity_W"]=triple(physical.payload_attachment.velocity);
    result["cable_direction_W"]=triple(physical.cable_direction_W);
    const auto relative=physical.payload_attachment.velocity-physical.drone_attachment.velocity;
    result["cable_angular_velocity_W"]=triple(physical.distance>0
        ? physical.cable_direction_W.cross(relative)/physical.distance : Vector3{});
    result["cable_distance"]=physical.distance;
    result["cable_radial_velocity"]=physical.radial_velocity;
    result["tension"]=physical.tension;
    result["energy"]=physical.energy;
    result["mode"]=state.slack ? "slack" : "taut";
    result["cable_force_on_drone_W"]=triple(physical.cable_direction_W*physical.tension);
    result["cable_force_on_payload_W"]=triple(physical.cable_direction_W*(-physical.tension));
    py::dict control; control["thrust"]=value.control.thrust; control["torque_B"]=triple(value.control.torque_B);
    result["control"]=control;
    py::dict air; air["drone"]=csim::binding::airLoads(physical.drone.aerodynamics);
    air["payload"]=csim::binding::airLoads(physical.payload.aerodynamics); result["aerodynamics"]=air;
    py::list events;
    double loss=0; Vector3 impulse;
    for (const auto& event:value.cable_events) {
        py::dict item;
        item["type"]=event.type; item["time"]=event.time;
        item["impulse_W"]=triple(event.impulse_W); item["energy_loss"]=event.energy_loss;
        item["radial_velocity_before"]=event.radial_velocity_before;
        item["radial_velocity_after"]=event.radial_velocity_after;
        item["mode_after"]=event.slack_after ? "slack" : "taut";
        events.append(item); loss+=event.energy_loss; impulse+=event.impulse_W;
    }
    result["cable_events"]=events; result["impact_energy_loss"]=loss; result["cable_impulse_W"]=triple(impulse);
    return result;
}
}

void bindRigidPayload(py::module_& module) {
    py::class_<RigidPayloadModel,std::shared_ptr<RigidPayloadModel>>(module,"RigidPayloadModel")
        .def_static("from_config",&modelFromConfig,py::arg("configuration"),
            "Restore aggregate physics from get_config; source links and visual geometry require the original model files.")
        .def_property_readonly("drone_mass",[](const RigidPayloadModel& model) { return model.physics().drone().mass(); })
        .def_property_readonly("payload_mass",[](const RigidPayloadModel& model) { return model.physics().payload().mass(); })
        .def_property_readonly("inertia_B",[](const RigidPayloadModel& model) { return inertia(model.physics().drone().inertia()); })
        .def_property_readonly("payload_inertia_P",[](const RigidPayloadModel& model) { return inertia(model.physics().payload().inertia()); })
        .def_property_readonly("drone_attachment_B",[](const RigidPayloadModel& model) { return triple(model.physics().droneAttachment()); })
        .def_property_readonly("payload_attachment_P",[](const RigidPayloadModel& model) { return triple(model.physics().payloadAttachment()); })
        .def_property_readonly("length",[](const RigidPayloadModel& model) { return model.physics().length(); })
        .def_property_readonly("gravity",[](const RigidPayloadModel& model) { return model.physics().drone().gravity(); })
        .def_property_readonly("timestep",&RigidPayloadModel::timestep)
        .def_property_readonly("cable_mode",[](const RigidPayloadModel& model) { return model.cable().mode; })
        .def_property_readonly("integrator",[](const RigidPayloadModel& model) { return model.integration().method; })
        .def_property_readonly("link_names",[](const RigidPayloadModel& model) { return csim::simulation::linkNames(model); });
    py::class_<RigidPayloadData>(module,"RigidPayloadData").def_property_readonly("model",&RigidPayloadData::model);
    module.def("load_suspended_model",[](const std::string& drone,const std::string& cable,const std::string& payload,
        double gravity,double timestep,const std::string& integrator,double rtol,double atol,std::size_t max_substeps,
        const Attitude& drone_rotation,const Attitude& payload_rotation,const std::map<std::string,std::string>& packages,
        const csim::dynamics::DragConfig& drone_drag,const csim::dynamics::DragConfig& payload_drag,const csim::dynamics::WindField& wind) {
        auto cable_config=loadCable(cable);
        auto drone_asset=loadBody(drone,drone_rotation,packages);
        auto payload_asset=loadBody(payload,payload_rotation,packages);
        return std::make_shared<RigidPayloadModel>(drone_asset,payload_asset,cable_config,gravity,timestep,
            csim::simulation::IntegratorSettings{integrator,rtol,atol,max_substeps},drone_drag,payload_drag,wind);
    },py::arg("drone_path"),py::arg("cable_path"),py::arg("payload_path"),py::kw_only(),
        py::arg("gravity")=9.80665,py::arg("timestep")=.001,py::arg("integrator")="rk4",
        py::arg("rtol")=1e-6,py::arg("atol")=1e-9,py::arg("max_substeps")=10000,
        py::arg("drone_root_to_body")=Attitude{1,0,0,0},py::arg("payload_root_to_body")=Attitude{1,0,0,0},
        py::arg("package_roots")=std::map<std::string,std::string>{},
        py::arg("drone_drag")=csim::dynamics::DragConfig{},py::arg("payload_drag")=csim::dynamics::DragConfig{},
        py::arg("wind")=csim::dynamics::WindField{},
        "Assemble two free rigid bodies from JSON/URDF/MJCF and a cable JSON. Each body requires cable_attachment.");
    module.def("make_data",[](std::shared_ptr<RigidPayloadModel> model,double thrust,const Triple& torque,const py::kwargs& fields) {
        if (!model) throw std::invalid_argument("Model must not be null");
        return RigidPayloadData(model,initialState(*model,fields),{thrust,vector(torque)});
    },py::arg("model"),py::arg("thrust")=0,py::arg("torque_B")=Triple{0,0,0},
        "Create rigid payload data. State fields are keyword arguments; default payload pose assembles the cable.");
    module.def("reset",[](const RigidPayloadModel& model,RigidPayloadData& data,double thrust,const Triple& torque,const py::kwargs& fields) {
        csim::simulation::checkModel(model,data);
        data.reset(initialState(model,fields),{thrust,vector(torque)});
    },py::arg("model"),py::arg("data"),py::arg("thrust")=0,py::arg("torque_B")=Triple{0,0,0});
    module.def("step",[](const RigidPayloadModel& model,RigidPayloadData& data) { csim::simulation::step(model,data); },
        py::arg("model"),py::arg("data"));
    module.def("step",[](const RigidPayloadModel& model,RigidPayloadData& data,double thrust,const Triple& torque) {
        csim::simulation::stepWithControl(model,data,{thrust,vector(torque)});
    },py::arg("model"),py::arg("data"),py::arg("thrust"),py::arg("torque_B")=Triple{});
    module.def("set_control",[](const RigidPayloadModel& model,RigidPayloadData& data,double thrust,const Triple& torque) {
        csim::simulation::setControl(model,data,{thrust,vector(torque)});
    },py::arg("model"),py::arg("data"),py::arg("thrust"),py::arg("torque_B")=Triple{0,0,0});
    module.def("get_state",[](const RigidPayloadModel& model,const RigidPayloadData& data) {
        return snapshot(csim::simulation::getState(model,data));
    },py::arg("model"),py::arg("data"));
}
