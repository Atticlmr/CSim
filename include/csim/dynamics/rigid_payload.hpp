#pragma once

#include <csim/dynamics/drone.hpp>
#include <csim/dynamics/suspended_payload.hpp>

namespace csim::dynamics {

struct RigidPayloadState {
    DroneState drone;
    DroneState payload;
    bool slack = false;
    bool isFinite() const { return drone.isFinite() && payload.isFinite(); }
};

inline RigidPayloadState operator+(const RigidPayloadState& left, const RigidPayloadState& right) {
    return {left.drone+right.drone,left.payload+right.payload,left.slack};
}
inline RigidPayloadState operator*(const RigidPayloadState& state, double scalar) {
    return {state.drone*scalar,state.payload*scalar,state.slack};
}

struct AttachmentKinematics {
    math::Vector3 position;
    math::Vector3 velocity;
};

struct RigidPayloadObservables {
    DroneObservables drone;
    DroneObservables payload;
    AttachmentKinematics drone_attachment;
    AttachmentKinematics payload_attachment;
    math::Vector3 cable_direction_W;
    double distance;
    double radial_velocity;
    double tension;
    double energy;
};

class RigidPayload {
public:
    RigidPayload(double drone_mass, math::Matrix3 drone_inertia,
                 double payload_mass, math::Matrix3 payload_inertia,
                 math::Vector3 drone_attachment, math::Vector3 payload_attachment,
                 double length, double gravity=9.80665,
                 DragConfig drone_drag={}, DragConfig payload_drag={}, WindField wind={})
        : drone_(drone_mass,drone_inertia,gravity,drone_drag,wind),
          payload_(payload_mass,payload_inertia,gravity,payload_drag,wind),
          drone_solver_(drone_.inertia()),payload_solver_(payload_.inertia()),
          drone_attachment_(drone_attachment),payload_attachment_(payload_attachment),length_(length) {
        if (!drone_attachment.isFinite() || !payload_attachment.isFinite()
            || !std::isfinite(length) || length<=0
            || !std::isfinite(drone_mass+payload_mass)) throw std::invalid_argument("Invalid rigid payload cable geometry or mass");
    }

    const Drone& drone() const { return drone_; }
    const Drone& payload() const { return payload_; }
    math::Vector3 droneAttachment() const { return drone_attachment_; }
    math::Vector3 payloadAttachment() const { return payload_attachment_; }
    double length() const { return length_; }

    static AttachmentKinematics attachment(const DroneState& state, math::Vector3 offset) {
        return {state.position_W+state.q_WB.rotate(offset),
            state.velocity_W+state.q_WB.rotate(state.angular_velocity_B.cross(offset))};
    }

    double distance(const RigidPayloadState& state) const {
        return (attachment(state.payload,payload_attachment_).position
               -attachment(state.drone,drone_attachment_).position).norm();
    }

    math::Vector3 direction(const RigidPayloadState& state) const {
        return (attachment(state.payload,payload_attachment_).position
               -attachment(state.drone,drone_attachment_).position).normalized();
    }

    double radialVelocity(const RigidPayloadState& state) const {
        return direction(state).dot(attachment(state.payload,payload_attachment_).velocity
                                  -attachment(state.drone,drone_attachment_).velocity);
    }

    double inverseEffectiveMass(const RigidPayloadState& state, math::Vector3 normal) const {
        const auto drone_axis=drone_attachment_.cross(state.drone.q_WB.conjugated().rotate(normal));
        const auto payload_axis=payload_attachment_.cross(state.payload.q_WB.conjugated().rotate(normal));
        const double result=1/drone_.mass()+1/payload_.mass()
            +drone_axis.dot(drone_solver_.solve(drone_axis))
            +payload_axis.dot(payload_solver_.solve(payload_axis));
        if (!std::isfinite(result) || result<=0) throw std::overflow_error("Invalid cable effective mass");
        return result;
    }

    double tension(const RigidPayloadState& state, const DroneControl& control, double time=0) const {
        const auto normal=direction(state);
        const auto drone_rate=drone_.derivative(state.drone,control,time);
        const auto payload_rate=payload_.derivative(state.payload,{},time);
        const auto acceleration=[](const DroneState& body, const DroneState& rate, math::Vector3 offset) {
            return rate.velocity_W+body.q_WB.rotate(rate.angular_velocity_B.cross(offset)
                +body.angular_velocity_B.cross(body.angular_velocity_B.cross(offset)));
        };
        const auto relative=attachment(state.payload,payload_attachment_).velocity
                           -attachment(state.drone,drone_attachment_).velocity;
        const double radial=relative.dot(normal);
        const double result=(normal.dot(acceleration(state.payload,payload_rate,payload_attachment_)
                             -acceleration(state.drone,drone_rate,drone_attachment_))
                            +(relative-normal*radial).squaredNorm()/distance(state))
                            /inverseEffectiveMass(state,normal);
        if (!std::isfinite(result)) throw std::overflow_error("Cable tension overflow");
        return result;
    }

    RigidPayloadState derivative(const RigidPayloadState& state, const DroneControl& control,
                                 double time=0, bool enforce=true) const {
        RigidPayloadState result{drone_.derivative(state.drone,control,time),
                                 payload_.derivative(state.payload,{},time),state.slack};
        if (!state.slack) {
            const double cable_tension=tension(state,control,time);
            if (enforce && cable_tension<=0) throw CableDomainError("Rigid payload taut cable has lost tension");
            const auto force=direction(state)*cable_tension;
            result.drone.velocity_W+=force/drone_.mass();
            result.payload.velocity_W-=force/payload_.mass();
            result.drone.angular_velocity_B+=drone_solver_.solve(
                drone_attachment_.cross(state.drone.q_WB.conjugated().rotate(force)));
            result.payload.angular_velocity_B-=payload_solver_.solve(
                payload_attachment_.cross(state.payload.q_WB.conjugated().rotate(force)));
        }
        if (!result.isFinite()) throw std::overflow_error("Rigid payload derivative overflow");
        return result;
    }

    void applyImpulse(RigidPayloadState& state, math::Vector3 impulse_W) const {
        state.drone.velocity_W+=impulse_W/drone_.mass();
        state.payload.velocity_W-=impulse_W/payload_.mass();
        state.drone.angular_velocity_B+=drone_solver_.solve(
            drone_attachment_.cross(state.drone.q_WB.conjugated().rotate(impulse_W)));
        state.payload.angular_velocity_B-=payload_solver_.solve(
            payload_attachment_.cross(state.payload.q_WB.conjugated().rotate(impulse_W)));
        if (!state.isFinite()) throw std::overflow_error("Rigid cable impulse overflow");
    }

    RigidPayloadState projected(RigidPayloadState state) const {
        state.drone=Drone::normalizedState(state.drone);
        state.payload=Drone::normalizedState(state.payload);
        if (!state.slack) {
            const auto normal=direction(state);
            const auto correction=normal*(distance(state)-length_);
            const double share=payload_.mass()/(drone_.mass()+payload_.mass());
            state.drone.position_W+=correction*share;
            state.payload.position_W-=correction*(1-share);
            applyImpulse(state,normal*(radialVelocity(state)/inverseEffectiveMass(state,normal)));
        }
        if (!state.isFinite()) throw std::overflow_error("Rigid cable projection overflow");
        return state;
    }

    RigidPayloadState validated(RigidPayloadState state) const {
        state.drone=Drone::normalizedState(state.drone);
        state.payload=Drone::normalizedState(state.payload);
        const double separation=distance(state);
        if (state.slack) {
            if (separation>length_*(1+1e-9)) throw std::invalid_argument("Slack attachment distance exceeds cable length");
        } else {
            if (std::abs(separation-length_)>1e-9*length_ || separation<=0)
                throw std::invalid_argument("Taut attachment distance must match cable length");
            const auto relative=attachment(state.payload,payload_attachment_).velocity
                               -attachment(state.drone,drone_attachment_).velocity;
            if (std::abs(radialVelocity(state))>1e-9*std::max(1.0,relative.norm()))
                throw std::invalid_argument("Taut attachment velocity violates cable constraint");
            state=projected(state);
        }
        return state;
    }

    RigidPayloadObservables observe(const RigidPayloadState& state, const DroneControl& control,
                                    double time=0, bool enforce=true) const {
        const auto rate=derivative(state,control,time,enforce);
        auto drone=drone_.observe(state.drone,control,time);
        auto payload=payload_.observe(state.payload,{},time);
        drone.acceleration_W=rate.drone.velocity_W;
        drone.angular_acceleration_B=rate.drone.angular_velocity_B;
        payload.acceleration_W=rate.payload.velocity_W;
        payload.angular_acceleration_B=rate.payload.angular_velocity_B;
        const double separation=distance(state);
        const auto normal=separation>0 ? direction(state) : math::Vector3{0,0,-1};
        const double radial=normal.dot(attachment(state.payload,payload_attachment_).velocity
                                      -attachment(state.drone,drone_attachment_).velocity);
        const double energy=drone.energy+payload.energy;
        if (!std::isfinite(energy)) throw std::overflow_error("Rigid payload energy overflow");
        return {drone,payload,attachment(state.drone,drone_attachment_),
            attachment(state.payload,payload_attachment_),normal,separation,radial,
            state.slack ? 0 : tension(state,control,time),energy};
    }

private:
    Drone drone_;
    Drone payload_;
    math::PartialPivLU<3> drone_solver_;
    math::PartialPivLU<3> payload_solver_;
    math::Vector3 drone_attachment_;
    math::Vector3 payload_attachment_;
    double length_;
};

} // namespace csim::dynamics
