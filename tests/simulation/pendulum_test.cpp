#include <csim/simulation/pendulum.hpp>

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
using namespace csim::simulation;
void check(bool c,const char* msg) { if (!c) throw std::runtime_error(msg); }
void near(double a,double b,double tol=1e-12) { check(std::isfinite(a)&&std::abs(a-b)<=tol,"numeric mismatch"); }
template <typename Error,typename F> void throws(F&& f) {
    try { f(); } catch (const Error&) { return; }
    throw std::runtime_error("expected exception missing");
}
void physicalStep() {
    auto model=std::make_shared<PendulumModel>(1,1,9.81,0.01);
    auto data=makeData(model,0.2);
    step(*model,data);
    auto s=getState(*model,data);
    const double a=9.81, theta=0.2, h=0.01;
    const double expected_angle=theta-0.5*a*std::sin(theta)*h*h
        +a*a*std::sin(theta)*std::cos(theta)*h*h*h*h/24;
    const double expected_rate=-a*std::sin(theta)*h
        +a*a*std::sin(theta)*std::cos(theta)*h*h*h/6;
    near(s.time,0.01,0);
    near(s.state.angle,expected_angle,1e-11);
    near(s.state.angular_velocity,expected_rate,1e-9);
    for(int i=1;i<100;++i) step(*model,data);
    near(getState(*model,data).time,1,2e-15);
    auto rest=makeData(model);
    for(int i=0;i<100;++i) step(*model,rest);
    const auto resting=getState(*model,rest);
    near(resting.state.angle,0,0);
    near(resting.state.angular_velocity,0,0);
    near(resting.physical.tension,9.81);
}
void snapshotsResetAndLifetime() {
    auto model=std::make_shared<PendulumModel>();
    auto data=makeData(model,0.3,-0.2);
    auto independent=makeData(model,-0.4);
    auto old=getState(*model,data);
    step(*model,data);
    near(old.time,0,0);
    old.state.angle=99;
    old.physical.position_W.x=99;
    check(getState(*model,data).state.angle!=99,"snapshot copy");
    near(getState(*model,independent).time,0,0);
    reset(*model,data,-0.1,0.2);
    const auto current=getState(*model,data);
    near(current.time,0,0);
    near(current.state.angle,-0.1,0);
    near(current.state.angular_velocity,0.2,0);
    std::weak_ptr<PendulumModel> weak=model;
    model.reset();
    check(!weak.expired(),"data retains immutable model");
    step(*data.model(),data);
    near(getState(*data.model(),data).time,0.001,0);
}
void modelIdentity() {
    auto a=std::make_shared<PendulumModel>();
    auto b=std::make_shared<PendulumModel>();
    auto data=makeData(a);
    throws<std::invalid_argument>([&]{step(*b,data);});
    throws<std::invalid_argument>([&]{getState(*b,data);});
    throws<std::invalid_argument>([&]{reset(*b,data);});
    throws<std::invalid_argument>([]{makeData(nullptr);});
    near(getState(*a,data).time,0,0);
}
void failedStepAndReset() {
    auto model=std::make_shared<PendulumModel>(1,1,9.81,2);
    auto data=makeData(model,0.7);
    const auto before=getState(*model,data);
    throws<csim::dynamics::PendulumDomainError>([&]{step(*model,data);});
    throws<csim::dynamics::PendulumDomainError>([&]{reset(*model,data,2);});
    const auto after=getState(*model,data);
    near(before.time,after.time,0);
    near(before.state.angle,after.state.angle,0);
    near(before.state.angular_velocity,after.state.angular_velocity,0);
    near(before.physical.energy,after.physical.energy,0);
    for(double bad:{0.0,-1.0,std::numeric_limits<double>::infinity()}) {
        throws<std::invalid_argument>([&]{PendulumModel p(1,1,9.81,bad);});
    }
    auto tiny=std::make_shared<PendulumModel>(1,1,9.81,std::numeric_limits<double>::denorm_min());
    auto tiny_data=makeData(tiny);
    throws<std::overflow_error>([&]{step(*tiny,tiny_data);});
    near(getState(*tiny,tiny_data).time,0,0);
    reset(*model,data);
    step(*model,data); // Equilibrium is valid even with this coarse step.
    near(getState(*model,data).time,2,0);
}
}
int main() {
    struct Test {const char* name;void(*run)();};
    const Test tests[]{{"physical step and time",physicalStep},{"snapshots reset and lifetime",snapshotsResetAndLifetime},
        {"model identity",modelIdentity},{"atomic step and reset failures",failedStepAndReset}};
    int failed=0;
    for(const auto& t:tests) {
        try {t.run();std::cout<<"PASS: "<<t.name<<'\n';}
        catch(const std::exception& e){++failed;std::cerr<<"FAIL: "<<t.name<<": "<<e.what()<<'\n';}
    }
    return failed?EXIT_FAILURE:EXIT_SUCCESS;
}
