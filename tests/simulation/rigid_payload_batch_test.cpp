#include <csim/simulation/rigid_payload_batch.hpp>

#include <array>
#include <iostream>
#include <thread>

using namespace csim;
using namespace csim::simulation;

void check(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance=1e-12) {
    check(std::isfinite(actual) && std::abs(actual-expected)<=tolerance, "Batch value mismatch");
}
std::shared_ptr<const model::RigidBodyAsset> asset(double mass, math::Matrix3 inertia,
                                                   math::Vector3 attachment) {
    auto result=std::make_shared<model::RigidBodyAsset>();
    result->mass=mass; result->inertia_B=inertia;
    result->attachments_B.emplace("cable_attachment",model::Pose{attachment,{}});
    return result;
}
std::shared_ptr<RigidPayloadModel> makeModel(const char* method="rk4", double timestep=.002,
                                            std::size_t max_substeps=10000) {
    return std::make_shared<RigidPayloadModel>(
        asset(1,dynamics::Drone::defaultInertia(),{.1,0,-.1}),
        asset(.5,math::Matrix3{.01,0,0,0,.015,0,0,0,.02},{.1,0,.1}),
        RigidCableSettings{},9.80665,timestep,
        IntegratorSettings{method,1e-6,1e-9,max_substeps});
}
void equal(const dynamics::RigidPayloadState& actual,const dynamics::RigidPayloadState& expected) {
    const auto compare=[](const dynamics::DroneState& left,const dynamics::DroneState& right) {
        return (left.position_W-right.position_W).norm()<1e-12
            && (left.velocity_W-right.velocity_W).norm()<1e-12
            && (left.q_WB-right.q_WB).norm()<1e-12
            && (left.angular_velocity_B-right.angular_velocity_B).norm()<1e-12;
    };
    check(compare(actual.drone,expected.drone) && compare(actual.payload,expected.payload)
          && actual.slack==expected.slack,"Rigid batch state mismatch");
}

int main() {
    try {
        for (const char* method : {"euler","midpoint","rk4","dopri5","dop853","lie_rk4"}) {
            auto model=makeModel(method);
            RigidPayloadBatch batch(model,7,3);
            std::vector<RigidPayloadData> scalar;
            std::vector<dynamics::RigidPayloadState> states;
            states.reserve(7); scalar.reserve(7);
            for (std::size_t index=0;index<7;++index) {
                auto state=model->initialState();
                state.slack=true;
                state.drone.position_W.x=static_cast<double>(index)*.01;
                state.payload.position_W.x=state.drone.position_W.x;
                states.push_back(state);
                scalar.emplace_back(model,state,dynamics::DroneControl{});
            }
            std::vector<std::size_t> indices{0,1,2,3,4,5,6};
            batch.reset(indices,states);
            for (std::size_t iteration=0;iteration<8;++iteration) {
                std::vector<dynamics::DroneControl> controls(7);
                for (std::size_t index=0;index<controls.size();++index)
                    controls[index]={15+static_cast<double>(index)*.01,{.01,-.02,.001*index}};
                batch.step(controls,2);
                const auto snapshots=batch.getState();
                for (std::size_t index=0;index<snapshots.size();++index) {
                    scalar[index].setControl(controls[index]);
                    std::size_t event_count=0;
                    scalar[index].step(); event_count+=scalar[index].getState().cable_events.size();
                    scalar[index].step(); event_count+=scalar[index].getState().cable_events.size();
                    const auto reference=scalar[index].getState();
                    near(snapshots[index].time,reference.time);
                    equal(snapshots[index].state,reference.state);
                    near(snapshots[index].control.thrust,reference.control.thrust);
                    check(snapshots[index].events.size()==event_count,"Batch event count mismatch");
                }
            }
        }
        std::cout<<"PASS: rigid batch parity across explicit, adaptive and Lie integrators\n";

        auto centered_model=std::make_shared<RigidPayloadModel>(
            asset(1,dynamics::Drone::defaultInertia(),{}),
            asset(.5,math::Matrix3{.01,0,0,0,.015,0,0,0,.02},{}),
            RigidCableSettings{});
        RigidPayloadBatch centered_batch(centered_model,7,3);
        centered_batch.step(std::vector<dynamics::DroneControl>(7,{1.5*9.80665,{}}));
        const auto centered_before=centered_batch.getState();
        centered_batch.step(std::vector<dynamics::DroneControl>(7),3);
        for (std::size_t index=0;index<7;++index) {
            const auto snapshot=centered_batch.getState()[index];
            check(snapshot.state.slack,"Control release did not enter slack mode");
            check(snapshot.events.size()==1 && snapshot.events.front().type=="release",
                  "Rigid batch dropped or duplicated control release");
            near(snapshot.events.front().time,centered_before[index].time);
        }
        centered_batch.step(std::vector<dynamics::DroneControl>(7));
        for (const auto& snapshot:centered_batch.getState())
            check(snapshot.events.empty(),"Rigid batch retained stale release events");
        std::cout<<"PASS: rigid batch reports control releases once across substeps\n";

        auto model=makeModel();
        RigidPayloadBatch batch(model,9,4);
        const auto before=batch.getState();
        std::vector<dynamics::DroneControl> controls(9,{15,{}});
        controls[8].thrust=-1;
        bool rejected=false;
        try { batch.step(controls); } catch (const std::invalid_argument&) { rejected=true; }
        check(rejected,"Invalid rigid batch control was accepted");
        const auto after=batch.getState();
        for (std::size_t index=0;index<before.size();++index) {
            near(after[index].time,before[index].time);
            equal(after[index].state,before[index].state);
        }
        std::cout<<"PASS: rigid batch validates controls before commit\n";

        auto failing_model=makeModel("rk4",.02,1);
        RigidPayloadBatch failing(failing_model,5,3);
        auto failure_states=failing.getState();
        controls.assign(5,{0,{}});
        bool failed=false;
        try { failing.step(controls); } catch (const numerics::IntegrationFailure&) { failed=true; }
        check(failed,"Rigid batch integration failure was not propagated");
        const auto recovered=failing.getState();
        for (std::size_t index=0;index<failure_states.size();++index) {
            near(recovered[index].time,failure_states[index].time);
            equal(recovered[index].state,failure_states[index].state);
            check(recovered[index].events.empty(),"Failed rigid batch leaked cable events");
        }
        std::cout<<"PASS: rigid batch cable-event failure is whole-batch atomic\n";

        auto concurrent_model=makeModel();
        RigidPayloadBatch concurrent(concurrent_model,32,4);
        const std::vector<dynamics::DroneControl> concurrent_controls(32,{15,{}});
        auto advance=[&] { for (int index=0;index<20;++index) concurrent.step(concurrent_controls,2); };
        std::thread first(advance),second(advance); first.join(); second.join();
        for (const auto& snapshot:concurrent.getState()) near(snapshot.time,.16,1e-12);
        std::cout<<"PASS: concurrent rigid batch calls serialize without partial snapshots\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr<<error.what()<<'\n'; return 1;
    }
}
