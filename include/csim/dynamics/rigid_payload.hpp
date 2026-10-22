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

    // One evaluation of the attachment geometry shared by every query below.
    // Without it, tension, projection and the observables each rebuild both
    // attachments several times for the same state.
    struct Kinematics {
        AttachmentKinematics drone;
        AttachmentKinematics payload;
        math::Vector3 normal;         // Drone attachment -> payload attachment.
        math::Vector3 relative_velocity;
        double distance = 0;
        double radial_velocity = 0;
        bool separated = false;       // distance > 0; normal is a fallback otherwise
    };

    Kinematics kinematics(const RigidPayloadState& state) const {
        const auto drone = attachment(state.drone, drone_attachment_);
        const auto payload = attachment(state.payload, payload_attachment_);
        const auto delta = payload.position - drone.position;
        const double separation = delta.norm();
        const bool separated = separation > 0;
        const auto normal = separated ? delta.normalized() : math::Vector3{0, 0, -1};
        const auto relative = payload.velocity - drone.velocity;
        return {drone, payload, normal, relative, separation, normal.dot(relative), separated};
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

    // Unconstrained taut tension. The free derivatives are evaluated here once and
    // shared by tensionFrom, so callers that also need the derivative or the
    // observables reuse the same two evaluations instead of repeating them.
    double tension(const RigidPayloadState& state, const DroneControl& control, double time=0) const {
        const auto free=freeRates(state,control,time);
        return tensionFrom(state,free,kinematics(state));
    }

    RigidPayloadState derivative(const RigidPayloadState& state, const DroneControl& control,
                                 double time=0, bool enforce=true) const {
        const auto free=freeRates(state,control,time);
        RigidPayloadState result{free.drone,free.payload,state.slack};
        if (!state.slack) {
            const auto kin=kinematics(state);
            const double cable_tension=tensionFrom(state,free,kin);
            if (enforce && cable_tension<=0) throw CableDomainError("Rigid payload taut cable has lost tension");
            applyCable(state,result,kin.normal,cable_tension);
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
            const auto kin=kinematics(state);
            requireSeparated(kin);
            const auto correction=kin.normal*(kin.distance-length_);
            const double share=payload_.mass()/(drone_.mass()+payload_.mass());
            state.drone.position_W+=correction*share;
            state.payload.position_W-=correction*(1-share);
            applyImpulse(state,kin.normal*(kin.radial_velocity/inverseEffectiveMass(state,kin.normal)));
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
        const auto free=freeRates(state,control,time);
        const auto kin=kinematics(state);
        RigidPayloadState rate{free.drone,free.payload,state.slack};
        double cable_tension=0;
        if (!state.slack) {
            cable_tension=tensionFrom(state,free,kin);
            if (enforce && cable_tension<=0) throw CableDomainError("Rigid payload taut cable has lost tension");
            applyCable(state,rate,kin.normal,cable_tension);
        }
        // The coupled rates already include the cable force, so the observables
        // reuse them instead of overwriting the body-only accelerations.
        auto drone=drone_.observeWith(state.drone,control,rate.drone,time);
        auto payload=payload_.observeWith(state.payload,{},rate.payload,time);
        const double energy=drone.energy+payload.energy;
        if (!std::isfinite(energy)) throw std::overflow_error("Rigid payload energy overflow");
        return {drone,payload,kin.drone,kin.payload,kin.normal,kin.distance,kin.radial_velocity,
            state.slack ? 0 : cable_tension,energy};
    }

    // Combined mechanical energy. Throws when either body's energy is not
    // representable; the per-step boundary check uses this instead of observe().
    double energy(const RigidPayloadState& state) const {
        return drone_.energyOf(state.drone) + payload_.energyOf(state.payload);
    }

    // Boundary check for callers that only need to know whether observe() would
    // succeed. Same checks in the same order; the reported vectors are skipped.
    void validateAt(const RigidPayloadState& state, const DroneControl& control,
                    double time=0, bool enforce=true) const {
        const auto free=freeRates(state,control,time);
        const auto kin=kinematics(state);
        RigidPayloadState rate{free.drone,free.payload,state.slack};
        if (!state.slack) {
            const double cable_tension=tensionFrom(state,free,kin);
            if (enforce && cable_tension<=0) throw CableDomainError("Rigid payload taut cable has lost tension");
            applyCable(state,rate,kin.normal,cable_tension);
        }
        if (!rate.isFinite()) throw std::overflow_error("Rigid payload derivative overflow");
        drone_.validateObservables(state.drone,control,time);
        payload_.validateObservables(state.payload,{},time);
    }

private:
    // Free (uncoupled) derivatives of both bodies. Tension and the coupled
    // derivative are both explicit functions of these, so they are evaluated once
    // per call site instead of once per consumer.
    struct FreeRates { DroneState drone; DroneState payload; };

    FreeRates freeRates(const RigidPayloadState& state,const DroneControl& control,double time) const {
        return {drone_.derivative(state.drone,control,time),payload_.derivative(state.payload,{},time)};
    }

    static math::Vector3 attachmentAcceleration(const DroneState& body,const DroneState& rate,
                                                math::Vector3 offset) {
        return rate.velocity_W+body.q_WB.rotate(rate.angular_velocity_B.cross(offset)
            +body.angular_velocity_B.cross(body.angular_velocity_B.cross(offset)));
    }

    double tensionFrom(const RigidPayloadState& state,const FreeRates& free,
                       const Kinematics& kin) const {
        requireSeparated(kin);
        const auto relative=kin.relative_velocity;
        const double radial=kin.radial_velocity;
        const double result=(kin.normal.dot(attachmentAcceleration(state.payload,free.payload,payload_attachment_)
                             -attachmentAcceleration(state.drone,free.drone,drone_attachment_))
                            +(relative-kin.normal*radial).squaredNorm()/kin.distance)
                            /inverseEffectiveMass(state,kin.normal);
        if (!std::isfinite(result)) throw std::overflow_error("Cable tension overflow");
        return result;
    }

    static void requireSeparated(const Kinematics& kin) {
        // Matches direction(): a zero separation has no cable axis. Only a slack
        // state can reach it, and no taut path below is called for one.
        if (!kin.separated) throw std::domain_error("Rigid cable direction is undefined at zero separation");
    }

    // Add the taut cable force to a derivative/rate pair. Same arithmetic as
    // applyImpulse, but the target holds accelerations instead of velocities.
    void applyCable(const RigidPayloadState& state,RigidPayloadState& rate,
                    math::Vector3 normal,double cable_tension) const {
        const auto force=normal*cable_tension;
        rate.drone.velocity_W+=force/drone_.mass();
        rate.payload.velocity_W-=force/payload_.mass();
        rate.drone.angular_velocity_B+=drone_solver_.solve(
            drone_attachment_.cross(state.drone.q_WB.conjugated().rotate(force)));
        rate.payload.angular_velocity_B-=payload_solver_.solve(
            payload_attachment_.cross(state.payload.q_WB.conjugated().rotate(force)));
    }

    Drone drone_;
    Drone payload_;
    math::PartialPivLU<3> drone_solver_;
    math::PartialPivLU<3> payload_solver_;
    math::Vector3 drone_attachment_;
    math::Vector3 payload_attachment_;
    double length_;
};

} // namespace csim::dynamics
