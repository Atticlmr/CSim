#include <csim/simulation/suspended_payload.hpp>
#include <iostream>

using namespace csim;
using namespace csim::simulation;
void check(bool ok) { if (!ok) throw std::runtime_error("Cable event regression failed"); }
void near(double a,double b,double tol=1e-9) { check(std::isfinite(a)&&std::abs(a-b)<tol); }

int main() {
    try {
        // Free-flight impact has known momentum and kinetic loss, including
        // tangential relative velocity. Angular momentum is unchanged because
        // equal/opposite impulses act along the line joining the two masses.
        dynamics::SuspendedPayload physics;
        dynamics::SuspendedPayloadState state;
        state.slack=true;
        state.drone.position_W={0,0,1};
        state.drone.velocity_W={.4,-.2,.3};
        state.payload_position_W={0,0,0};
        state.payload_velocity_W={.6,.1,-.7};
        auto result=cable_detail::advance(physics,{}, {},state,{},0,1e-5);
        check(result.events.size()==1);
        const auto& event=result.events.front();
        near(event.time,0); near(event.energy_loss,1.0/12);
        near(event.impulse_W.z,-1.0/6); near(event.radial_velocity_before,1);
        const auto final=physics.observe(result.state,{},1e-5,false);
        const auto momentum0=state.drone.velocity_W+state.payload_velocity_W*.2;
        const auto momentum1=result.state.drone.velocity_W+final.payload_velocity_W*.2;
        near((momentum1-momentum0-math::Vector3{0,0,-1.2*physics.drone().gravity()*1e-5}).norm(),0);
        const auto angular0=state.drone.position_W.cross(state.drone.velocity_W)
            +state.payload_position_W.cross(state.payload_velocity_W*.2);
        const auto angular1=result.state.drone.position_W.cross(result.state.drone.velocity_W)
            +final.payload_position_W.cross(final.payload_velocity_W*.2);
        const auto com0=(state.drone.position_W+state.payload_position_W*.2)/1.2;
        const auto gravity=math::Vector3{0,0,-1.2*physics.drone().gravity()};
        const auto gravity_moment=(com0*1e-5+momentum0/1.2*(.5e-10)).cross(gravity);
        near((angular1-angular0-gravity_moment).norm(),0);
        std::cout<<"PASS: impulse momentum, angular momentum and energy loss\n";

        // A step which has already computed an impact must roll back if the
        // remaining continuous integration exceeds its work budget.
        auto model=std::make_shared<SuspendedPayloadModel>(1,.2,1,dynamics::Drone::defaultInertia(),
            9.80665,.02,nullptr,dynamics::DragConfig{},dynamics::DragConfig{},dynamics::WindField{},
            IntegratorSettings{"rk4",1e-6,1e-9,1},"hybrid");
        auto data=makeData(model,{},state);
        const auto before=getState(*model,data);
        bool failed=false;
        try { step(*model,data); } catch (const numerics::IntegrationFailure&) { failed=true; }
        check(failed);
        const auto after=getState(*model,data);
        near(after.time,before.time);
        near((after.state.drone.velocity_W-before.state.drone.velocity_W).norm(),0);
        near((after.state.payload_velocity_W-before.state.payload_velocity_W).norm(),0);
        check(after.cable_events.empty()); near(after.physical.cable_impulse_W.norm(),0);
        std::cout<<"PASS: post-impact failure is atomic\n";
        return 0;
    } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
