#pragma once

#include <csim/dynamics/drone.hpp>
#include <algorithm>

namespace csim::dynamics {

class CableDomainError : public std::domain_error {
public:
    using std::domain_error::domain_error;
};

// Massless cable attached at the drone centre of mass. In taut mode the
// direction/angular velocity are the reduced S^2 coordinates. In slack mode
// payload_position/velocity are independent Cartesian states and the cable
// exerts no force until the distance reaches length.
struct SuspendedPayloadState {
    DroneState drone{};
    math::Vector3 cable_direction_W{0, 0, -1};
    math::Vector3 cable_angular_velocity_W{};
    math::Vector3 payload_position_W{};
    math::Vector3 payload_velocity_W{};
    bool slack=false;

    bool isFinite() const {
        return drone.isFinite() && cable_direction_W.isFinite()
            && cable_angular_velocity_W.isFinite() && payload_position_W.isFinite()
            && payload_velocity_W.isFinite();
    }
};

inline SuspendedPayloadState operator+(const SuspendedPayloadState& a,
                                       const SuspendedPayloadState& b) noexcept {
    return {a.drone + b.drone, a.cable_direction_W + b.cable_direction_W,
            a.cable_angular_velocity_W + b.cable_angular_velocity_W,
            a.payload_position_W + b.payload_position_W,
            a.payload_velocity_W + b.payload_velocity_W, a.slack};
}
inline SuspendedPayloadState operator*(const SuspendedPayloadState& s, double value) noexcept {
    return {s.drone * value, s.cable_direction_W * value, s.cable_angular_velocity_W * value,
            s.payload_position_W * value, s.payload_velocity_W * value, s.slack};
}

struct SuspendedPayloadObservables {
    DroneObservables drone;
    math::Vector3 payload_position_W;
    math::Vector3 payload_velocity_W;
    math::Vector3 payload_acceleration_W;
    math::Vector3 cable_force_on_drone_W;
    math::Vector3 cable_force_on_payload_W;
    math::Vector3 cable_angular_acceleration_W;
    double tension;
    double energy;
    AirLoads payload_aerodynamics;
    bool slack=false;
    double cable_distance=0;
    double cable_radial_velocity=0;
    math::Vector3 cable_impulse_W{};
};

class SuspendedPayload {
public:
    SuspendedPayload(double drone_mass = 1, double payload_mass = 0.2, double length = 1,
                     math::Matrix3 inertia_B = Drone::defaultInertia(), double gravity = 9.80665,
                     DragConfig drone_drag = {}, DragConfig payload_drag = {}, WindField wind = {})
        : drone_(drone_mass, inertia_B, gravity, drone_drag, wind), payload_mass_(positive(payload_mass)),
          length_(positive(length)), payload_drag_(std::move(payload_drag)) {
        payload_drag_.validate();
        const double total_mass = drone_.mass() + payload_mass_;
        reduced_mass_ = drone_.mass() * (payload_mass_ / total_mass);
        if (!std::isfinite(total_mass) || !std::isfinite(reduced_mass_) || reduced_mass_ <= 0)
            throw std::overflow_error("Coupled masses cannot be represented");
    }

    const Drone& drone() const noexcept { return drone_; }
    const DragConfig& payloadDrag() const noexcept { return payload_drag_; }
    double payloadMass() const noexcept { return payload_mass_; }
    double length() const noexcept { return length_; }

    AirLoads payloadAirLoads(const SuspendedPayloadState& state,double time=0) const {
        const auto position=state.slack ? state.payload_position_W
            : state.drone.position_W+state.cable_direction_W*length_;
        const auto velocity=state.slack ? state.payload_velocity_W
            : state.drone.velocity_W+state.cable_angular_velocity_W.cross(state.cable_direction_W)*length_;
        return payload_drag_.forces(velocity,drone_.wind().at(position,time));
    }

    // Normalize a public state. Taut states must satisfy the S^2 constraints;
    // slack states must have a finite payload and no distance upper-bound clamp.
    static SuspendedPayloadState normalizedState(SuspendedPayloadState state, double length=1) {
        if (!state.isFinite()) throw std::invalid_argument("Coupled state must be finite");
        state.drone = Drone::normalizedState(state.drone);
        if (!state.slack) {
            const double n=state.cable_direction_W.norm(), w=state.cable_angular_velocity_W.norm();
            if (!std::isfinite(n)||!std::isfinite(w)||std::abs(n-1)>1e-9
                || std::abs(state.cable_direction_W.dot(state.cable_angular_velocity_W))
                    > 1e-9*std::max(1.0,w))
                throw std::invalid_argument("Cable direction must be unit and angular velocity tangent");
            state.cable_direction_W=state.cable_direction_W.normalized();
            state.cable_angular_velocity_W-=state.cable_direction_W*state.cable_direction_W.dot(state.cable_angular_velocity_W);
            state.payload_position_W={};
            state.payload_velocity_W={};
        } else {
            const auto delta=state.payload_position_W-state.drone.position_W;
            const double distance=delta.norm();
            if (!std::isfinite(distance)) throw std::overflow_error("Cable distance is not representable");
            if (distance>length*(1+1e-9)) throw std::invalid_argument("Slack cable state exceeds cable length");
            if (distance>1e-12) {
                state.cable_direction_W=delta/distance;
                const auto relative=state.payload_velocity_W-state.drone.velocity_W;
                state.cable_angular_velocity_W=state.cable_direction_W.cross(relative)/distance;
            } else {
                state.cable_direction_W={0,0,-1}; state.cable_angular_velocity_W={};
            }
        }
        return state;
    }

    static SuspendedPayloadState projectedState(SuspendedPayloadState state, double length=1) {
        if (!state.isFinite()) throw std::invalid_argument("Coupled state must be finite");
        state.drone=Drone::normalizedState(state.drone);
        if (state.slack) {
            const auto delta=state.payload_position_W-state.drone.position_W;
            const double distance=delta.norm();
            if (distance>1e-12) {
                state.cable_direction_W=delta/distance;
                state.cable_angular_velocity_W=state.cable_direction_W.cross(
                    state.payload_velocity_W-state.drone.velocity_W)/distance;
            } else { state.cable_direction_W={0,0,-1}; state.cable_angular_velocity_W={}; }
        } else {
            state.cable_direction_W=state.cable_direction_W.normalized();
            state.cable_angular_velocity_W-=state.cable_direction_W*state.cable_direction_W.dot(state.cable_angular_velocity_W);
            state.payload_position_W={};
            state.payload_velocity_W={};
        }
        if (!state.isFinite()) throw std::overflow_error("Cable projection overflowed");
        return state;
    }

    // Derivative permits a negative taut tension so the simulation layer can
    // detect a release event and switch to slack instead of throwing mid-trial.
    SuspendedPayloadState derivative(const SuspendedPayloadState& input,
                                     const DroneControl& control, double time=0,
                                     bool enforce_tension=true) const {
        auto state=input;
        if (!state.isFinite()) throw std::invalid_argument("Coupled state must be finite");
        if (state.slack) {
            state=projectedState(state,length_);
            // As in standalone flight, q-dot uses the raw RK stage quaternion;
            // normalization belongs to the accepted state and force rotation.
            state.drone.q_WB=input.drone.q_WB;
        }
        else {
            state.cable_direction_W=state.cable_direction_W.normalized();
            state.cable_angular_velocity_W-=state.cable_direction_W
                *state.cable_direction_W.dot(state.cable_angular_velocity_W);
        }
        auto result=state; result.slack=state.slack;
        auto drone_derivative=drone_.derivative(state.drone,control,time);
        if (state.slack) {
            const auto air=payloadAirLoads(state,time);
            auto payload_acc=math::Vector3{0,0,-drone_.gravity()};
            if (payload_drag_.enabled()) payload_acc+=air.total_force_W/payload_mass_;
            result.drone=drone_derivative;
            result.payload_position_W=state.payload_velocity_W;
            result.payload_velocity_W=payload_acc;
            result.cable_direction_W={}; result.cable_angular_velocity_W={};
        } else {
            const auto cable=forces(state,control,time,enforce_tension);
            drone_derivative.velocity_W+=cable.force/drone_.mass();
            result.drone=drone_derivative;
            result.cable_direction_W=state.cable_angular_velocity_W.cross(state.cable_direction_W);
            result.cable_angular_velocity_W=cable.alpha;
            result.payload_position_W={}; result.payload_velocity_W={};
        }
        if (!result.isFinite()) throw std::overflow_error("Coupled derivative overflowed");
        return result;
    }

    SuspendedPayloadObservables observe(const SuspendedPayloadState& input,
                                        const DroneControl& control, double time=0,
                                        bool enforce_tension=true) const {
        const auto valid=normalizedState(input,length_);
        const auto payload_position=valid.slack ? valid.payload_position_W
            : valid.drone.position_W+valid.cable_direction_W*length_;
        const auto payload_velocity=valid.slack ? valid.payload_velocity_W
            : valid.drone.velocity_W+valid.cable_angular_velocity_W.cross(valid.cable_direction_W)*length_;
        const auto separation=payload_position-valid.drone.position_W;
        const double distance=separation.norm();
        const auto direction=distance>1e-12 ? separation/distance : math::Vector3{0,0,-1};
        if (!std::isfinite(distance) || (!valid.slack && (distance <= 0 ||
            ((separation/length_)-valid.cable_direction_W).norm()>1e-10)))
            throw std::overflow_error("World coordinates cannot resolve cable geometry accurately");
        const double radial=distance>1e-12 ? (payload_velocity-valid.drone.velocity_W).dot(direction) : 0;
        const auto payload_air=payloadAirLoads(valid,time);
        auto drone=drone_.observe(valid.drone,control,time);
        math::Vector3 force_drone{},force_payload{},payload_acc{0,0,-drone_.gravity()};
        double tension=0;
        math::Vector3 alpha{};
        if (!valid.slack) {
            const auto cable=forces(valid,control,time,enforce_tension); tension=cable.tension; force_drone=cable.force; force_payload=-cable.force; alpha=cable.alpha;
            drone.acceleration_W+=force_drone/drone_.mass();
            payload_acc-=force_drone/payload_mass_;
        }
        if (payload_drag_.enabled()) payload_acc+=payload_air.total_force_W/payload_mass_;
        const double energy=drone.energy+0.5*payload_mass_*payload_velocity.squaredNorm()
            +payload_mass_*drone_.gravity()*payload_position.z;
        if (!drone.acceleration_W.isFinite()||!payload_position.isFinite()||!payload_velocity.isFinite()
            ||!payload_acc.isFinite()||!std::isfinite(energy)) throw std::overflow_error("Coupled observables overflowed");
        return {drone,payload_position,payload_velocity,payload_acc,force_drone,force_payload,
            alpha,tension,energy,payload_air,valid.slack,distance,radial,{}};
    }

    // Evaluate the unconstrained taut tension. Used by the hybrid event handler.
    double tautTension(const SuspendedPayloadState& state,const DroneControl& control,double time=0) const {
        const auto valid=projectedState(state,length_);
        return forces(valid,control,time,false).tension;
    }

    private:
    struct CableForces { double tension; math::Vector3 force; math::Vector3 alpha; };
    CableForces forces(const SuspendedPayloadState& state,const DroneControl& control,double time,bool enforce) const {
        Drone::validateControl(control);
        const auto& s=state.cable_direction_W;
        const auto ds=state.cable_angular_velocity_W.cross(s);
        const double thrust_acceleration=control.thrust/drone_.mass();
        if (!std::isfinite(thrust_acceleration)) throw std::overflow_error("Thrust acceleration overflowed");
        const auto b=state.drone.q_WB.rotate({0,0,thrust_acceleration});
        double tension=reduced_mass_*(length_*ds.squaredNorm()-s.dot(b));
        auto alpha=-s.cross(b)/length_;
        if (drone_.drag().enabled()||payload_drag_.enabled()) {
            auto relative=-b;
            if (drone_.drag().enabled()) relative-=drone_.airLoads(state.drone,time).total_force_W/drone_.mass();
            if (payload_drag_.enabled()) relative+=payloadAirLoads(state,time).total_force_W/payload_mass_;
            tension=reduced_mass_*(length_*ds.squaredNorm()+s.dot(relative)); alpha=s.cross(relative)/length_;
        }
        const auto force=s*tension;
        if (!std::isfinite(tension)||!force.isFinite()||!alpha.isFinite()) throw std::overflow_error("Cable force overflowed");
        if (enforce&&tension<=0) throw CableDomainError("Taut cable has lost tension");
        return {tension,force,alpha};
    }
    static double positive(double value) {
        if (!std::isfinite(value)||value<=0) throw std::invalid_argument("Payload mass and cable length must be finite and positive");
        return value;
    }
    Drone drone_; double payload_mass_; double length_; double reduced_mass_; DragConfig payload_drag_;
};
} // namespace csim::dynamics
