#pragma once

#include <csim/dynamics/rigid_payload.hpp>
#include <csim/model/rigid_body.hpp>
#include <csim/simulation/cable_events.hpp>

namespace csim::simulation {

struct RigidCableSettings {
    double length = 1;
    std::string mode = "hybrid";
    math::Vector3 initial_direction_W{0,0,-1};
    CableEventSettings events;
    void validate() const {
        if (!std::isfinite(length) || length<=0 || (mode!="taut" && mode!="hybrid"))
            throw std::invalid_argument("Require positive cable length and mode taut or hybrid");
        if (!initial_direction_W.isFinite() || std::abs(initial_direction_W.norm()-1)>1e-9)
            throw std::invalid_argument("Cable initial_direction_W must be a unit vector");
        events.validate();
    }
};

inline math::Vector3 cableAttachment(const model::RigidBodyAsset& asset) {
    const auto attachment=asset.attachments_B.find("cable_attachment");
    for (const auto& link:asset.links) {
        if (link.name=="cable_attachment") {
            if (attachment!=asset.attachments_B.end())
                throw std::invalid_argument("Ambiguous cable_attachment: both link and site defined");
            return link.body_from_link.position;
        }
    }
    if (attachment!=asset.attachments_B.end()) return attachment->second.position;
    throw std::invalid_argument("Body description requires cable_attachment link or site");
}

class RigidPayloadModel {
public:
    RigidPayloadModel(std::shared_ptr<const model::RigidBodyAsset> drone,
                      std::shared_ptr<const model::RigidBodyAsset> payload,
                      RigidCableSettings cable, double gravity=9.80665, double timestep=.001,
                      IntegratorSettings integration={}, dynamics::DragConfig drone_drag={},
                      dynamics::DragConfig payload_drag={}, dynamics::WindField wind={})
        : drone_asset_(requireAsset(std::move(drone))),payload_asset_(requireAsset(std::move(payload))),
          cable_(std::move(cable)),timestep_(timestep),integration_(std::move(integration)),
          physics_(drone_asset_->mass,drone_asset_->inertia_B,payload_asset_->mass,payload_asset_->inertia_B,
              cableAttachment(*drone_asset_),cableAttachment(*payload_asset_),cable_.length,
              gravity,drone_drag,payload_drag,wind) {
        cable_.validate();
        integration_.validate();
        if (!std::isfinite(timestep_) || timestep_<=0) throw std::invalid_argument("Require positive timestep");
    }
    const auto& physics() const { return physics_; }
    const auto& asset() const { return drone_asset_; }
    const auto& payloadAsset() const { return payload_asset_; }
    const auto& cable() const { return cable_; }
    const auto& integration() const { return integration_; }
    double timestep() const { return timestep_; }
    bool hybridCable() const { return cable_.mode=="hybrid"; }
    dynamics::RigidPayloadState initialState() const {
        dynamics::RigidPayloadState state;
        state.drone.position_W=drone_asset_->initial_pose_WB.position;
        state.drone.q_WB=drone_asset_->initial_pose_WB.orientation;
        state.payload.q_WB=payload_asset_->initial_pose_WB.orientation;
        state.payload.position_W=dynamics::RigidPayload::attachment(state.drone,physics_.droneAttachment()).position
            +cable_.initial_direction_W.normalized()*cable_.length
            -state.payload.q_WB.rotate(physics_.payloadAttachment());
        return state;
    }
private:
    static std::shared_ptr<const model::RigidBodyAsset> requireAsset(std::shared_ptr<const model::RigidBodyAsset> asset) {
        if (!asset) throw std::invalid_argument("Rigid payload requires two body assets");
        return asset;
    }
    const std::shared_ptr<const model::RigidBodyAsset> drone_asset_;
    const std::shared_ptr<const model::RigidBodyAsset> payload_asset_;
    const RigidCableSettings cable_;
    const double timestep_;
    const IntegratorSettings integration_;
    const dynamics::RigidPayload physics_;
};

namespace rigid_cable_detail {
inline dynamics::RigidPayloadState admissible(const RigidPayloadModel& model,
        dynamics::RigidPayloadState state, const dynamics::DroneControl& control, double time) {
    if (state.slack && !model.hybridCable()) throw std::invalid_argument("Slack state requires hybrid cable");
    if (model.hybridCable() && !state.slack && model.physics().tension(state,control,time)<=0)
        state.slack=true;
    return state;
}

struct Result {
    dynamics::RigidPayloadState state;
    std::vector<CableEvent> events;
};

inline Result advance(const RigidPayloadModel& model, dynamics::RigidPayloadState state,
                      const dynamics::DroneControl& control, double start) {
    numerics::detail::validateExplicitTime(start,model.timestep());
    const auto& physics=model.physics();
    const auto& settings=model.cable().events;
    const double end=start+model.timestep();
    std::size_t work=0;
    auto propagate=[&](const dynamics::RigidPayloadState& initial,double time,double interval) {
        return integration_detail::advance(model.integration(),time,initial,interval,
            [&](double stage,const dynamics::RigidPayloadState& value,int) {
                return physics.derivative(value,control,stage,!model.hybridCable());
            },
            [&](dynamics::RigidPayloadState value) { return physics.projected(value); },
            [&](const dynamics::RigidPayloadState& value,double time_value) {
                if (++work>model.integration().max_substeps)
                    throw numerics::IntegrationFailure("Rigid cable exceeded max_substeps");
                if (!model.hybridCable()) (void)physics.observe(value,control,time_value);
                numerics::detail::checkComputedState(value);
            });
    };
    if (!model.hybridCable()) return {propagate(state,start,model.timestep()),{}};
    Result result{state,{}};
    auto append=[&](CableEvent event) {
        if (result.events.size()>=settings.max_events)
            throw numerics::IntegrationFailure("Rigid cable event limit exceeded");
        result.events.push_back(std::move(event));
    };
    auto guard=[&](const dynamics::RigidPayloadState& value,double time) {
        const double result_value=value.slack ? physics.distance(value)-physics.length()
                                             : -physics.tension(value,control,time);
        if (!std::isfinite(result_value)) throw std::overflow_error("Rigid cable guard overflow");
        return result_value;
    };
    auto impact=[&](dynamics::RigidPayloadState& value,double time) {
        const auto normal=physics.direction(value);
        const double radial=physics.radialVelocity(value);
        if (radial<-1e-10) throw numerics::IntegrationFailure("Rigid cable crossing is not outward");
        const double impulse=std::max(0.0,radial)/physics.inverseEffectiveMass(value,normal);
        const auto correction=normal*(physics.distance(value)-physics.length());
        const double share=physics.payload().mass()/(physics.drone().mass()+physics.payload().mass());
        value.drone.position_W+=correction*share;
        value.payload.position_W-=correction*(1-share);
        physics.applyImpulse(value,normal*impulse);
        value.slack=false;
        value=physics.projected(value);
        value=admissible(model,value,control,time);
        const double loss=.5*impulse*std::max(0.0,radial);
        if (!std::isfinite(loss)) throw std::overflow_error("Rigid cable impact loss overflow");
        append({"impact",time,normal*impulse,loss,radial,physics.radialVelocity(value),value.slack});
    };
    const double epsilon=64*std::numeric_limits<double>::epsilon()*physics.length();
    double time=start;
    while (time<end) {
        auto& current=result.state;
        if (!current.slack && physics.tension(current,control,time)<=0) {
            current.slack=true;
            append({"release",time,{},0,0,0,true});
        }
        if (current.slack && physics.distance(current)>=physics.length()-epsilon) {
            const double radial=physics.radialVelocity(current);
            if (radial>1e-10 || (std::abs(radial)<=1e-10 && physics.tension(current,control,time)>0))
                impact(current,time);
        }
        double target=std::min(end,time+settings.max_step);
        if (end-target<=8*std::numeric_limits<double>::epsilon()*std::max(std::abs(end),std::abs(time))) target=end;
        if (!(target>time)) throw numerics::IntegrationFailure("Rigid cable time cannot advance");
        auto candidate=propagate(current,time,target-time);
        const double before=guard(current,time),after=guard(candidate,target);
        const bool crossing=current.slack ? (after>epsilon || (before<-epsilon && after>=0)) : after>=0;
        if (!crossing) { current=candidate; time=target; continue; }
        double lower=time,upper=target;
        while (upper-lower>settings.time_tolerance) {
            const double middle=lower+(upper-lower)*.5;
            if (!(middle>lower && middle<upper)) throw numerics::IntegrationFailure("Rigid cable event time unresolved");
            auto trial=propagate(current,time,middle-time);
            if (guard(trial,middle)>=0) { upper=middle; candidate=trial; }
            else lower=middle;
        }
        current=candidate;
        time=upper;
        if (current.slack) impact(current,time);
        else {
            current.slack=true;
            append({"release",time,{},0,0,0,true});
        }
    }
    (void)physics.observe(result.state,control,end,false);
    return result;
}
}

struct RigidPayloadSnapshot {
    double time;
    dynamics::RigidPayloadState state;
    dynamics::DroneControl control;
    dynamics::RigidPayloadObservables physical;
    std::vector<CableEvent> cable_events;
};

class RigidPayloadData {
public:
    RigidPayloadData(std::shared_ptr<RigidPayloadModel> model, dynamics::RigidPayloadState state,
                     dynamics::DroneControl control)
        : model_(std::move(model)) { reset(state,control); }
    std::shared_ptr<RigidPayloadModel> model() const { return model_; }
    RigidPayloadSnapshot getState() const {
        return {time_,state_,control_,model_->physics().observe(state_,control_,time_,!model_->hybridCable()),events_};
    }
    void setControl(dynamics::DroneControl control) {
        auto candidate=rigid_cable_detail::admissible(*model_,state_,control,time_);
        (void)model_->physics().observe(candidate,control,time_,!model_->hybridCable());
        const bool released=!state_.slack && candidate.slack;
        auto events=events_;
        if (released) events.push_back({"release",time_,{},0,0,0,true});
        events_=std::move(events);
        pending_release_=pending_release_ || released;
        state_=candidate;
        control_=control;
    }
    void step() {
        auto candidate=rigid_cable_detail::advance(*model_,state_,control_,time_);
        if (pending_release_)
            cable_detail::prependControlRelease(candidate.events,time_,model_->cable().events.max_events);
        state_=candidate.state;
        events_=std::move(candidate.events);
        pending_release_=false;
        time_+=model_->timestep();
    }
    void reset(dynamics::RigidPayloadState state, dynamics::DroneControl control) {
        if (!model_) throw std::invalid_argument("Rigid payload model must not be null");
        auto candidate=model_->physics().validated(state);
        candidate=rigid_cable_detail::admissible(*model_,candidate,control,0);
        (void)model_->physics().observe(candidate,control,0,!model_->hybridCable());
        state_=candidate;
        control_=control;
        time_=0;
        events_.clear();
        pending_release_=false;
    }
private:
    std::shared_ptr<RigidPayloadModel> model_;
    dynamics::RigidPayloadState state_;
    dynamics::DroneControl control_;
    std::vector<CableEvent> events_;
    bool pending_release_=false;
    double time_=0;
};

inline void checkModel(const RigidPayloadModel& model, const RigidPayloadData& data) {
    if (&model!=data.model().get()) throw std::invalid_argument("Data belongs to a different model");
}
inline RigidPayloadSnapshot getState(const RigidPayloadModel& model, const RigidPayloadData& data) {
    checkModel(model,data); return data.getState();
}
inline void step(const RigidPayloadModel& model, RigidPayloadData& data) {
    checkModel(model,data); data.step();
}
inline void setControl(const RigidPayloadModel& model, RigidPayloadData& data, dynamics::DroneControl control) {
    checkModel(model,data); data.setControl(control);
}
inline void reset(const RigidPayloadModel& model, RigidPayloadData& data,
                  dynamics::RigidPayloadState state, dynamics::DroneControl control) {
    checkModel(model,data); data.reset(state,control);
}

} // namespace csim::simulation
