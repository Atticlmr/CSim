#include <csim/numerics/explicit_runge_kutta.hpp>
#include <csim/numerics/symplectic.hpp>
#include <csim/math/vector.hpp>
#include <csim/math/quaternion.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
using namespace csim::numerics;
void check(bool yes,const char* message) { if (!yes) throw std::runtime_error(message); }
void near(double a,double b,double tolerance=1e-12) { check(std::isfinite(a)&&std::abs(a-b)<=tolerance,"numeric mismatch"); }
template<class Error,class F> void throws(F&& f) {
    try { f(); } catch (const Error&) { return; } throw std::runtime_error("expected exception");
}
template<class F> auto explicitStep(int method,double t,double x,double h,F&& f) {
    if (method==0) return eulerStep(t,x,3.,h,f);
    if (method==1) return midpointStep(t,x,3.,h,f);
    return heunStep(t,x,3.,h,f);
}
void stages() {
    for (int method=0;method<3;++method) {
        struct Dynamics {
            std::vector<double> times;
            Dynamics()=default;
            Dynamics(const Dynamics&)=delete;
            double operator()(double t,double x,const double& u) & { times.push_back(t); return t+x+u; }
        } dynamics;
        near(explicitStep(method,1.,2.,.5,dynamics),method==0 ? 5. : 5.875);
        check(dynamics.times==(method==0 ? std::vector<double>{1.} :
            method==1 ? std::vector<double>{1.,1.25} : std::vector<double>{1.,1.5}),"stage times");
    }
}
void convergence() {
    for (int method=0;method<3;++method) {
        double previous=0;
        for (int n:{20,40,80}) {
            const double h=1./n; double x=1;
            for (int i=0;i<n;++i) x=explicitStep(method,i*h,x,h,[](double t,double x,double){ return x+t; });
            const double error=std::abs(x-(2*std::exp(1.)-2));
            if (previous) check(previous/error>(method==0?1.9:3.8)&&previous/error<(method==0?2.1:4.2),"convergence order");
            previous=error;
        }
    }
}
void mechanicalConvergence() {
    for (bool verlet:{false,true}) {
        double previous=0;
        for (int n:{20,40,80}) {
            const double dt=1./n; double q=1,v=0;
            for (int i=0;i<n;++i) {
                auto force=[](double q){ return -q; };
                const auto next=verlet ? velocityVerletStep(q,v,dt,force) : symplecticEulerStep(q,v,dt,force);
                q=next.first; v=next.second;
            }
            const double error=std::hypot(q-std::cos(1.),v+std::sin(1.));
            if (previous) check(previous/error>(verlet?3.8:1.9)&&previous/error<(verlet?4.2:2.1),"mechanical order");
            previous=error;
        }
    }
}
void mechanicalEnergyAndReversal() {
    for (bool verlet:{false,true}) {
        double q=1,v=0,max_error=0;
        auto force=[](double q){return -q;};
        for (int i=0;i<100000;++i) {
            const auto next=verlet ? velocityVerletStep(q,v,.1,force) : symplecticEulerStep(q,v,.1,force);
            q=next.first; v=next.second; max_error=std::max(max_error,std::abs((q*q+v*v)/2-.5));
        }
        check(max_error<(verlet?.0013:.027),"bounded oscillator energy error");
        if (verlet) {
            v=-v;
            for (int i=0;i<100000;++i) { const auto next=velocityVerletStep(q,v,.1,force); q=next.first; v=next.second; }
            near(q,1,1e-11); near(v,0,1e-11);
        }
    }
}
void stateTypes() {
    using csim::math::Vector3;
    const Vector3 q{1,2,3},v{2,-1,0};
    const auto next=velocityVerletStep(q,v,.5,[](const Vector3&){return Vector3{0,0,-10};});
    near(next.first.x,2); near(next.first.y,1.5); near(next.first.z,1.75); near(next.second.z,-5);
    using csim::math::Quaternion;
    const auto zero=[](double,const Quaternion&,int){return Quaternion{0,0,0,0};};
    near(eulerStep(0.,Quaternion{},0,.1,zero).w,1);
    near(midpointStep(0.,Quaternion{},0,.1,zero).w,1);
    near(heunStep(0.,Quaternion{},0,.1,zero).w,1);
}
void failures() {
    const double inf=std::numeric_limits<double>::infinity();
    for (int method=0;method<3;++method) {
        throws<std::invalid_argument>([&]{explicitStep(method,0,1,0,[](double,double,double){return 0.;});});
        throws<std::overflow_error>([&]{explicitStep(method,1e20,1,.001,[](double,double,double){return 0.;});});
        throws<std::overflow_error>([&]{explicitStep(method,0,1,.1,[&](double,double,double){return inf;});});
        throws<std::domain_error>([&]{explicitStep(method,0,1,.1,[](double,double,double)->double{throw std::domain_error("domain");});});
    }
    throws<std::invalid_argument>([]{velocityVerletStep(1.,0.,-1.,[](double){return 0.;});});
    throws<std::overflow_error>([&]{symplecticEulerStep(1.,0.,1.,[&](double){return inf;});});
}
}
int main() {
    try {
        stages(); std::cout<<"PASS: explicit stages and noncopyable dynamics\n";
        convergence(); std::cout<<"PASS: nonautonomous first/second order convergence\n";
        mechanicalConvergence(); std::cout<<"PASS: mechanical first/second order convergence\n";
        mechanicalEnergyAndReversal(); std::cout<<"PASS: long-time bounded energy and Verlet reversal\n";
        stateTypes(); std::cout<<"PASS: vector and quaternion state arithmetic\n";
        failures(); std::cout<<"PASS: invalid steps, overflow and domain propagation\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
