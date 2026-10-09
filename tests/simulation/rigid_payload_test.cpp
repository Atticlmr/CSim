#include <csim/simulation/rigid_payload.hpp>
#include <iostream>

using namespace csim;
using namespace csim::simulation;

void check(bool value) { if (!value) throw std::runtime_error("Rigid payload regression failed"); }
void near(double actual,double expected,double tolerance=1e-10) {
    check(std::isfinite(actual) && std::abs(actual-expected)<tolerance);
}
std::shared_ptr<const model::RigidBodyAsset> asset(double mass,math::Matrix3 inertia,math::Vector3 offset) {
    auto result=std::make_shared<model::RigidBodyAsset>();
    result->mass=mass; result->inertia_B=inertia;
    result->attachments_B.emplace("cable_attachment",model::Pose{offset,{}});
    return result;
}
math::Vector3 momentum(const dynamics::RigidPayload& physics,const dynamics::RigidPayloadState& state) {
    return state.drone.velocity_W*physics.drone().mass()+state.payload.velocity_W*physics.payload().mass();
}
math::Vector3 angularMomentum(const dynamics::RigidPayload& physics,const dynamics::RigidPayloadState& state) {
    return state.drone.position_W.cross(state.drone.velocity_W*physics.drone().mass())
        +state.payload.position_W.cross(state.payload.velocity_W*physics.payload().mass())
        +state.drone.q_WB.rotate(physics.drone().inertia()*state.drone.angular_velocity_B)
        +state.payload.q_WB.rotate(physics.payload().inertia()*state.payload.angular_velocity_B);
}

int main() {
    try {
        const auto drone_inertia=dynamics::Drone::defaultInertia();
        const math::Matrix3 payload_inertia{.01,0,0,0,.015,0,0,0,.02};
        dynamics::RigidPayload physics(1,drone_inertia,.5,payload_inertia,{.1,0,-.1},{.1,0,.1},1);
        dynamics::RigidPayloadState state;
        state.drone.position_W={0,0,2}; state.payload.position_W={0,0,.8};
        state=physics.validated(state);
        const auto normal=physics.direction(state);
        const double inverse=3+.01/.02+.01/.015;
        near(physics.inverseEffectiveMass(state,normal),inverse);
        near(physics.tension(state,{15,{}}),15/inverse);
        const auto rate=physics.derivative(state,{15,{}});
        near(rate.drone.angular_velocity_B.y,.1*(15/inverse)/.02);
        near(rate.payload.angular_velocity_B.y,-.1*(15/inverse)/.015);
        const auto acceleration_drone=rate.drone.velocity_W+rate.drone.angular_velocity_B.cross(physics.droneAttachment());
        const auto acceleration_payload=rate.payload.velocity_W+rate.payload.angular_velocity_B.cross(physics.payloadAttachment());
        near(normal.dot(acceleration_payload-acceleration_drone),0);
        std::cout<<"PASS: offset force moments and rotational effective mass\n";

        state.slack=true; state.drone.velocity_W={.3,.2,.1}; state.payload.velocity_W={-.2,.1,-.9};
        state.drone.angular_velocity_B={.2,-.1,.3}; state.payload.angular_velocity_B={-.1,.3,.2};
        const auto initial_momentum=momentum(physics,state),initial_angular=angularMomentum(physics,state);
        const double initial_energy=physics.observe(state,{},0,false).energy;
        const double radial=physics.radialVelocity(state);
        const double impulse=radial/physics.inverseEffectiveMass(state,normal);
        physics.applyImpulse(state,normal*impulse);
        near(physics.radialVelocity(state),0);
        near((momentum(physics,state)-initial_momentum).norm(),0);
        near((angularMomentum(physics,state)-initial_angular).norm(),0);
        near(initial_energy-physics.observe(state,{},0,false).energy,.5*impulse*radial);
        std::cout<<"PASS: offset impact conserves linear/angular momentum and correct energy loss\n";

        dynamics::RigidPayload centered(1,drone_inertia,.5,payload_inertia,{},{},1);
        dynamics::SuspendedPayload point(1,.5,1,drone_inertia);
        dynamics::RigidPayloadState rigid;
        rigid.drone.position_W={0,0,2}; rigid.payload.position_W={0,0,1}; rigid.payload.velocity_W={.4,0,0};
        dynamics::SuspendedPayloadState point_state;
        point_state.drone=rigid.drone; point_state.cable_direction_W={0,0,-1};
        point_state.cable_angular_velocity_W={0,-.4,0};
        const auto point_observation=point.observe(point_state,{15,{}});
        const auto rigid_observation=centered.observe(rigid,{15,{}});
        near(rigid_observation.tension,point_observation.tension);
        near((rigid_observation.drone.acceleration_W-point_observation.drone.acceleration_W).norm(),0);
        near((rigid_observation.payload.acceleration_W-point_observation.payload_acceleration_W).norm(),0);
        std::cout<<"PASS: zero-offset dynamics reduce to the point-payload model\n";

        auto hover_model=std::make_shared<RigidPayloadModel>(asset(1,drone_inertia,{0,0,-.1}),
            asset(.5,payload_inertia,{0,0,.1}),RigidCableSettings{},9.80665,.002);
        RigidPayloadData hover(hover_model,hover_model->initialState(),{1.5*9.80665,{}});
        const auto initial=hover.getState();
        for (int count=0;count<1000;++count) hover.step();
        const auto final=hover.getState();
        near((final.state.drone.position_W-initial.state.drone.position_W).norm(),0);
        near((final.state.payload.position_W-initial.state.payload.position_W).norm(),0);
        near(final.physical.tension,.5*9.80665);
        near(final.physical.energy,initial.physical.energy);
        std::cout<<"PASS: centered attachment hover and cable constraint\n";

        hover.setControl({0,{}});
        check(hover.getState().cable_events.size()==1);
        hover.step();
        const auto released=hover.getState();
        check(released.state.slack && released.cable_events.size()==1);
        check(released.cable_events.front().type=="release");
        near(released.cable_events.front().time,final.time);
        hover.step();
        check(hover.getState().cable_events.empty());
        std::cout<<"PASS: input-triggered release survives one step without duplicate events\n";

        RigidCableSettings settings; settings.events.max_step=.005;
        auto event_model=std::make_shared<RigidPayloadModel>(asset(1,drone_inertia,{.1,0,-.1}),
            asset(.5,payload_inertia,{.1,0,.1}),settings,9.80665,.02);
        auto event_state=event_model->initialState();
        event_state.slack=true; event_state.payload.position_W.z+=.01;
        event_state.payload.velocity_W.z=-1;
        RigidPayloadData event_data(event_model,event_state,{});
        event_data.step();
        const auto after_event=event_data.getState();
        check(!after_event.cable_events.empty());
        const auto& event=after_event.cable_events.front();
        check(event.type=="impact"); near(event.time,.01,2e-9);
        near(event.energy_loss,.5/inverse,2e-9); near(event.radial_velocity_after,0);
        check(after_event.physical.distance<=1+1e-10);
        check(after_event.state.drone.angular_velocity_B.norm()>0);
        check(after_event.state.payload.angular_velocity_B.norm()>0);
        std::cout<<"PASS: located slack-to-taut impact spins both rigid bodies\n";

        auto rollback_model=std::make_shared<RigidPayloadModel>(asset(1,drone_inertia,{.1,0,-.1}),
            asset(.5,payload_inertia,{.1,0,.1}),settings,9.80665,.02,IntegratorSettings{"rk4",1e-6,1e-9,1});
        event_state=rollback_model->initialState(); event_state.slack=true; event_state.payload.velocity_W.z=-1;
        RigidPayloadData rollback(rollback_model,event_state,{});
        const auto before=rollback.getState();
        bool failed=false;
        try { rollback.step(); } catch (const numerics::IntegrationFailure&) { failed=true; }
        check(failed);
        const auto after=rollback.getState();
        near(after.time,before.time);
        near((after.state.drone.velocity_W-before.state.drone.velocity_W).norm(),0);
        near((after.state.payload.angular_velocity_B-before.state.payload.angular_velocity_B).norm(),0);
        check(after.cable_events.empty());
        std::cout<<"PASS: failed integration rolls back impacts and both states\n";

        settings.mode="taut";
        double previous_error=0;
        for (const double timestep:{.04,.02,.01}) {
            auto convergence_model=std::make_shared<RigidPayloadModel>(asset(1,drone_inertia,{.1,0,-.1}),
                asset(.5,payload_inertia,{.1,0,.1}),settings,9.80665,timestep);
            auto initial_state=convergence_model->initialState();
            initial_state.drone.position_W.z+=3; initial_state.payload.position_W.z+=3;
            initial_state.drone.angular_velocity_B={.1,.4,.2};
            initial_state.payload.angular_velocity_B={-.1,.5,.1};
            initial_state.payload.velocity_W={.71,.01,.01};
            RigidPayloadData convergence(convergence_model,initial_state,{});
            const auto reference=convergence.getState();
            for (int count=0;count<std::lround(.8/timestep);++count) convergence.step();
            const auto observation=convergence.getState();
            const double error=std::abs(observation.physical.energy-reference.physical.energy);
            if (previous_error>0) check(error<previous_error/10);
            previous_error=error;
            const auto& coupled=convergence_model->physics();
            const auto initial_linear=momentum(coupled,reference.state);
            const auto center=(reference.state.drone.position_W+reference.state.payload.position_W*.5)/1.5;
            const math::Vector3 gravity_force{0,0,-1.5*coupled.drone().gravity()};
            near((momentum(coupled,observation.state)-initial_linear-gravity_force*.8).norm(),0,1e-10);
            const auto gravity_moment=(center*.8+initial_linear/1.5*(.5*.8*.8)).cross(gravity_force);
            near((angularMomentum(coupled,observation.state)-angularMomentum(coupled,reference.state)-gravity_moment).norm(),0,1e-7);
            near(observation.physical.distance,1);
            near(observation.physical.radial_velocity,0);
        }
        std::cout<<"PASS: spatial motion momentum balances and fourth-order energy convergence\n";
        return 0;
    } catch (const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
