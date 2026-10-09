#include <csim/simulation/integration.hpp>
#include <csim/numerics/rattle.hpp>
#include <iostream>

using namespace csim;
void check(bool value) { if (!value) throw std::runtime_error("Advanced method regression failed"); }
void near(double actual,double expected,double tolerance=1e-10) { check(std::isfinite(actual)&&std::abs(actual-expected)<tolerance); }
int main() {
    try {
        double previous_error=0;
        auto growth=[](double,double value,int) { return value; };
        for (double interval:{1.,.5,.25}) {
            const auto trial=numerics::dop853Trial(0,1.,0,interval,growth);
            const double error=std::abs(trial.state-std::exp(interval));
            if (previous_error>0) check(error<previous_error/200);
            previous_error=error;
        }
        double previous_non_autonomous=0;
        for (double interval:{.5,.25,.125}) {
            const auto trial=numerics::dop853Trial(.3,1.,0,interval,[](double time,double value,int) { return time*value; });
            const double error=std::abs(trial.state-std::exp((std::pow(.3+interval,2)-.09)/2));
            if (previous_non_autonomous>0) check(error<previous_non_autonomous/100);
            previous_non_autonomous=error;
        }
        bool collapsed_time=false;
        try { numerics::dop853Trial(1e16,1.,0,22.,growth); }
        catch (const std::overflow_error&) { collapsed_time=true; }
        check(collapsed_time);
        numerics::Dop853<double> high_order(0,1,1);
        const numerics::ScalarErrorNorm accurate(1e-11,1e-13);
        bool saw_rejection=false;
        while (high_order.time()<1) {
            const auto report=high_order.step(0,growth,accurate,1);
            saw_rejection=saw_rejection || report.rejected>0;
        }
        near(high_order.state(),std::exp(1),1e-11); check(saw_rejection);
        numerics::Dop853<double> exhausted(0,1,1,{1e-12,1,1});
        bool budget_rejected=false;
        try { exhausted.step(0,growth,accurate,1); }
        catch (const numerics::IntegrationFailure&) { budget_rejected=true; }
        check(budget_rejected); near(exhausted.time(),0); near(exhausted.state(),1); near(exhausted.nextStepSize(),1);
        bool invalid_norm=false;
        try { exhausted.step(0,growth,[](double,double,double) { return -1.; },1); }
        catch (const std::invalid_argument&) { invalid_norm=true; }
        check(invalid_norm); near(exhausted.time(),0);
        std::cout<<"PASS: DOP853 eighth-order solution, adaptive rejection, boundaries and atomic failure\n";
        const numerics::NewtonOptions tight{1e-14,0,20};
        auto decay=[](double,double state,int) { return -state; };
        for (const std::string method:{"implicit_euler","implicit_midpoint","radau5","bdf1","bdf2"}) {
            double previous=0;
            for (const double timestep:{.25,.125,.0625}) {
                double state=1,time=0;
                numerics::Bdf<double> bdf(0,1,method=="bdf1" ? 1 : 2,tight);
                for (int count=0;count<std::lround(1/timestep);++count) {
                    if (method=="radau5") state=numerics::RadauIIA5<double>(tight).step(time,state,0,timestep,decay);
                    else if (method=="bdf1" || method=="bdf2") { bdf.step(0,timestep,decay); state=bdf.state(); }
                    else state=numerics::implicitThetaStep(time,state,0,timestep,decay,method=="implicit_euler" ? 1 : .5,tight);
                    time+=timestep;
                }
                const double error=std::abs(state-std::exp(-1));
                const double ratio=method=="radau5" ? 20 : method=="implicit_euler" || method=="bdf1" ? 1.7 : 3.2;
                if (previous>0) check(error<previous/ratio);
                previous=error;
            }
        }
        auto stiff=[](double time,double state,int) { return -1000*(state-std::cos(time))-std::sin(time); };
        double state=1;
        for (int count=0;count<50;++count) state=numerics::RadauIIA5<double>(tight).step(count*.02,state,0,.02,stiff);
        near(state,std::cos(1),1e-9);
        near(numerics::RadauIIA5<double>(tight).step(0,1.,0,1.,[](double,double value,int) { return -1000000*value; }),0,4e-6);
        std::cout<<"PASS: implicit method orders, stiff forcing and Radau L-stability\n";

        numerics::Bdf<double> failed(0,0,1);
        bool rejected=false;
        try { failed.step(0,1.,[](double,double value,int) { return value*value+1; }); }
        catch (const numerics::IntegrationFailure&) { rejected=true; }
        check(rejected); near(failed.time(),0); near(failed.state(),0);
        failed.reset(2,1); failed.step(0,.1,decay); near(failed.time(),2.1); near(failed.state(),1/1.1);
        std::cout<<"PASS: failed Newton is atomic and BDF reset clears history\n";

        dynamics::Drone physics(1,math::Matrix3{.02,0,0,0,.03,0,0,0,.04});
        dynamics::DroneState initial; initial.angular_velocity_B={1,2,3};
        auto dynamics=[&](double time,const auto& value,int) { return physics.derivative(value,{},time); };
        auto reference=initial;
        for (int count=0;count<4000;++count) reference=numerics::rk4Step(count*.0001,reference,0,.0001,dynamics);
        for (bool fourth:{false,true}) {
            double previous=0;
            for (double timestep:{.04,.02,.01}) {
                auto value=initial;
                for (int count=0;count<std::lround(.4/timestep);++count)
                    value=numerics::LieGroupIntegrator<dynamics::DroneState,simulation::integration_detail::LieGeometry<dynamics::DroneState>>::step(
                        count*timestep,value,0,timestep,dynamics,fourth);
                const double error=(value.q_WB+reference.q_WB*(-1)).norm()+(value.angular_velocity_B-reference.angular_velocity_B).norm();
                if (previous>0) check(error<previous/(fourth ? 12 : 3.5));
                previous=error; near(value.q_WB.norm(),1);
            }
        }
        initial.angular_velocity_B={0,0,5};
        const auto rotation=numerics::LieGroupIntegrator<dynamics::DroneState,simulation::integration_detail::LieGeometry<dynamics::DroneState>>::step(0,initial,0,.5,dynamics);
        near(rotation.q_WB.w,std::cos(1.25)); near(rotation.q_WB.z,std::sin(1.25));
        std::cout<<"PASS: Lie midpoint/RKMK4 noncommuting rotation orders and exact constant-rate attitude\n";

        using Vector=math::Matrix<2,1>;
        numerics::Rattle<2> rattle(Vector{1,1});
        auto force=[](double,const Vector&) { return Vector{0,-9.80665}; };
        auto constraint=[](const Vector& value) { return math::Matrix<1,1>{.5*(value(0,0)*value(0,0)+value(1,0)*value(1,0)-1)}; };
        auto jacobian=[](const Vector& value) { return math::Matrix<1,2>{value(0,0),value(1,0)}; };
        Vector position{std::sin(.3),-std::cos(.3)},momentum{};
        const auto initial_position=position,initial_momentum=momentum;
        const auto energy=[](const Vector& position,const Vector& momentum) {
            return .5*(momentum(0,0)*momentum(0,0)+momentum(1,0)*momentum(1,0))+9.80665*position(1,0);
        };
        const double initial_energy=energy(position,momentum);
        double maximum_error=0;
        for (int count=0;count<10000;++count) {
            const auto result=rattle.step(count*.002,position,momentum,.002,force,constraint,jacobian);
            position=result.first; momentum=result.second;
            near(constraint(position)(0,0),0,2e-12); near((jacobian(position)*momentum)(0,0),0);
            maximum_error=std::max(maximum_error,std::abs(energy(position,momentum)-initial_energy));
        }
        check(maximum_error<5e-6);
        momentum*= -1;
        for (int count=0;count<10000;++count) {
            const auto result=rattle.step(count*.002,position,momentum,.002,force,constraint,jacobian);
            position=result.first; momentum=result.second;
        }
        near(position(0,0),initial_position(0,0),1e-9); near(position(1,0),initial_position(1,0),1e-9);
        near(momentum(0,0),-initial_momentum(0,0),1e-9);
        std::cout<<"PASS: RATTLE position/momentum constraints, long-time bounded energy and reversal\n";
        return 0;
    } catch (const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
